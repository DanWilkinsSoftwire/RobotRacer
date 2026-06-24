#!/usr/bin/env python3
"""Maze runner — STEP 2: line-follow + always-turn-left at junctions.

Policy: the **left-hand rule**. At every junction that offers a left path, take
it. This solves any simply-connected maze (every wall connected to the outer
boundary). It won't find the shortest path and can loop forever on a maze with
islands — that's the known trade-off for v1.

Two decoupled loops (the architecture from junction_detector.py):
    - VISION worker thread (imported, unchanged): classifies what's ahead into a
      shared JunctionState at ~8 Hz.
    - This FAST control loop (~50 Hz): grayscale line-following + motors. It
      *reads* the shared state but never blocks on the camera.

Fusion — why both sensors:
    Three grayscale sensors sit a few cm apart and can't tell a left *curve*
    from a left *junction*. The camera sees the junction coming. So we commit a
    left turn only when BOTH agree:
        vision label says a junction with a left path is ahead, AND
        the left grayscale sensor reads tape right now ("I'm on it").

Run it (claim the robot first!):
    ./deploy.sh run experimentation/maze_runner.py
    Ctrl + C to stop. Motors stop on exit.
"""

import os
import sys
import threading
from time import monotonic, sleep

# Repo root on the path so `from config import CONFIG` works when run from
# experimentation/ (sys.path[0] is already this dir, so junction_detector imports too).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from picarx import Picarx
from vilib import Vilib

from config import CONFIG
from junction_detector import JunctionState, vision_worker

# --- Tuning knobs ------------------------------------------------------------
FOLLOW_HZ = 50                 # control loop rate
DRIVE_POWER = CONFIG["drive_power"]
STEER_OFFSET = CONFIG["steer_offset"]   # gentle correction while following
TURN_ANGLE = 30                # sharp steer for a committed junction turn ([-30,30], + = left)
CAM_TILT_ANGLE = -30           # tilt camera DOWN at the floor ([-35,65], negative = down).
                               # Keeps the view on near floor/tape, not the distant horizon,
                               # so we stop reading clutter as junctions.
MIN_TURN_S = 0.4               # don't declare the turn "done" before this
MAX_TURN_S = 2.5               # give up turning after this (avoid spinning forever)
COOLDOWN_S = 1.0               # after a turn, ignore junctions briefly so we don't re-fire

# Vision labels that mean "a junction with a possible LEFT path is ahead".
# branch_right is excluded: no left path there, so we just keep following.
JUNCTION_LABELS = {"fork_or_T", "cross_or_multi", "branch_left", "horizontal_bar"}


def read_on_tape(px, reference):
    """[left, mid, right] booleans: True where a sensor sees white tape.

    White tape reads HIGH (brighter -> larger), so on_tape = value > reference.
    We compute this ourselves rather than use get_line_status(), whose 0=line
    convention is inverted for white-on-dark (see docs/picar-api.md).
    """
    data = px.get_grayscale_data()
    return [v > r for v, r in zip(data, reference)], data


def follow_steer(on_tape, last_steer):
    """Steering angle for normal line-following from the on-tape booleans.

    Signs and forward-priority are copied from light_line_tracking.py, which
    tracks our white line reliably on this robot. The servo sign is
    calibration-dependent (see docs/picar-api.md), so we trust the file that
    empirically works rather than re-derive it — the previous version steered
    the opposite way. Forward-priority: as long as the MID sensor sees the line
    we go straight, and only correct once the line has slipped to an outer
    sensor. That's far less twitchy than correcting whenever an outer sensor is
    also on.
    """
    left, mid, right = on_tape
    if mid:
        return 0                       # line under centre -> straight
    if left:
        return -STEER_OFFSET           # line slipped to the left sensor -> steer toward it
    if right:
        return STEER_OFFSET            # line slipped to the right sensor -> steer toward it
    return last_steer                  # line lost -> hold last correction


def main():
    px = Picarx()
    reference = CONFIG["line_reference"]
    px.set_line_reference(reference)
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(CAM_TILT_ANGLE)   # look down at the floor in front

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)   # watch at http://<robot-ip>:9000/mjpg
    sleep(0.8)

    # Set camera angle AFTER Vilib startup — Vilib re-inits the PWM board, which
    # snaps servos back to neutral, so an earlier tilt would just flick and reset.
    px.set_cam_pan_angle(0)
    px.set_cam_tilt_angle(CAM_TILT_ANGLE)   # look down at the floor in front

    state = JunctionState()
    stop_event = threading.Event()
    worker = threading.Thread(target=vision_worker, args=(state, stop_event),
                              daemon=True)
    worker.start()

    period = 1.0 / FOLLOW_HZ
    mode = "follow"           # "follow" | "turn_left"
    last_steer = 0
    turn_start = 0.0
    cooldown_until = 0.0

    print("Maze runner: line-follow + always-left. Ctrl+C to stop.\n")
    try:
        px.forward(DRIVE_POWER)
        while True:
            now = monotonic()
            on_tape, raw = read_on_tape(px, reference)
            js = state.snapshot()

            if mode == "follow":
                junction_left = (js["label"] in JUNCTION_LABELS and on_tape[0])
                if now >= cooldown_until and junction_left:
                    mode = "turn_left"
                    turn_start = now
                    px.set_dir_servo_angle(TURN_ANGLE)
                    px.forward(DRIVE_POWER)
                    print(f"-> JUNCTION ({js['label']}) raw={raw}: turning left")
                else:
                    steer = follow_steer(on_tape, last_steer)
                    last_steer = steer
                    px.set_dir_servo_angle(steer)
                    px.forward(DRIVE_POWER)

            elif mode == "turn_left":
                elapsed = now - turn_start
                centred = on_tape == [False, True, False]
                if elapsed >= MIN_TURN_S and centred:
                    mode = "follow"             # reacquired the line on the left branch
                    cooldown_until = now + COOLDOWN_S
                    last_steer = 0
                    px.set_dir_servo_angle(0)
                    print(f"   turn complete in {elapsed:.1f}s")
                elif elapsed >= MAX_TURN_S:
                    mode = "follow"             # bailout: never found the line
                    cooldown_until = now + COOLDOWN_S
                    last_steer = 0
                    px.set_dir_servo_angle(0)
                    print(f"   turn TIMED OUT after {elapsed:.1f}s — resuming follow")
                # else: keep turning (steer/power already commanded)

            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        px.stop()
        Vilib.camera_close()
        print("\nstopped")


if __name__ == "__main__":
    main()
