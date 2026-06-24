#!/usr/bin/env python3
"""Route runner — follow the line, execute a PREDEFINED turn at each junction.

Detection (robust to this maze's varying / low-contrast surface):
    AdaptiveLine decides "on tape" from real-time contrast, not a fixed
    threshold. A junction is detected when BOTH outer sensors read tape at once
    (a + / T crossbar -> [T,T,T]; a Y / fork -> [T,F,T]). Normal line-following
    never lights both outer sensors at once, so this is a clean junction signal.

Choice:
    ROUTE (route.py) is an ordered list of actions, one consumed per junction.
    We just COUNT junctions — the Nth junction runs ROUTE[N]. No type checking.

Turns use a PIVOT (tank-turn): the rear motors are independent, so we spin the
robot roughly in place (rear wheels opposite + front wheels steered into the
turn) until the new branch is centred under the sensors. Tighter and more
reliable for 90 degrees than an Ackermann arc, and it makes U-turns possible.

Run it (claim the robot!):
    ./deploy.sh run experimentation/route_runner.py LLFRLF
    # L=left  R=right  F=forward(straight)  U=uturn  S=stop
    # omit the string to use ROUTE in route.py
    Ctrl+C to stop. Motors stop on exit.

No camera here — pure grayscale, so it can run alongside grayscale_server.py
(open http://<robot-ip>:9001/ to watch the sensors while it drives).
"""

import argparse
import os
import sys
from time import monotonic, sleep

# Repo root on the path so `from config import CONFIG` works when run from
# experimentation/ (sys.path[0] is already this dir, so sibling imports work too).
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from picarx import Picarx

from config import CONFIG
from adaptive_line import AdaptiveLine
from route import ROUTE

# --- Tuning knobs ------------------------------------------------------------
FOLLOW_HZ = 50
DRIVE_POWER = CONFIG["drive_power"]
STEER_OFFSET = CONFIG["steer_offset"]       # gentle correction while following
TURN_ANGLE = 30             # front-wheel steer into a pivot ([-30,30], + = left)
PIVOT_POWER = CONFIG.get("pivot_power", DRIVE_POWER)
CREEP_S = 0.15              # creep forward this long before pivoting, to centre the
                           # wheelbase over the junction (sensors sit ahead of the
                           # rear pivot axis). Set 0 to disable.
MIN_TURN_S = 0.4           # ignore line reacquisition before this (skip the crossbar we're on)
MAX_TURN_S = 2.5           # bail out of a 90-degree pivot after this
MIN_UTURN_S = 1.2          # a 180 takes longer; don't finish early
MAX_UTURN_S = 4.0
COOLDOWN_S = 1.0           # after handling a junction, ignore junctions briefly
                           # (also carries a "straight" across the crossbar)

# Single-letter route actions, e.g. "LLFRLF".
ACTION_BY_CHAR = {
    "L": "left",
    "R": "right",
    "F": "straight",       # forward
    "U": "uturn",
    "S": "stop",
}


def parse_route(s):
    """Parse a route string like "LLFRLF" -> ["left","left","straight",...].

    Case-insensitive; spaces, commas, dashes and underscores are ignored as
    separators so "LL F R-L F" also works.
    """
    actions = []
    for ch in s:
        if ch in " ,-_":
            continue
        key = ch.upper()
        if key not in ACTION_BY_CHAR:
            raise SystemExit("route_runner: bad route char %r — use L/R/F/U/S" % ch)
        actions.append(ACTION_BY_CHAR[key])
    return actions


def pivot(px, direction, power):
    """direction: +1 = left (CCW), -1 = right (CW). Rear-wheel tank-turn in place.

    Signs follow picarx forward() = (motor1:+, motor2:-). So:
        left  (CCW) = right wheel fwd + left wheel back = both motors negative
        right (CW)  = left wheel fwd + right wheel back = both motors positive
    If your robot pivots the WRONG way, swap the two motor signs below (a motor
    calibration / wiring difference).
    """
    px.set_dir_servo_angle(direction * TURN_ANGLE)   # front wheels into the turn
    if direction > 0:        # left
        px.set_motor_speed(1, -power)
        px.set_motor_speed(2, -power)
    else:                    # right
        px.set_motor_speed(1, power)
        px.set_motor_speed(2, power)


def follow_steer(on_tape, last_steer):
    """Steering angle for normal line-following from the on-tape booleans."""
    left, mid, right = on_tape
    if mid and not left and not right:
        return 0                       # centred
    if left and not right:
        return STEER_OFFSET            # tape to the left -> steer left
    if right and not left:
        return -STEER_OFFSET           # tape to the right -> steer right
    if not any(on_tape):
        return last_steer              # lost the line -> hold last correction (recovery)
    return 0                           # ambiguous (e.g. on a crossbar) -> straight


def main():
    parser = argparse.ArgumentParser(
        description="PiCar-X predefined-route maze runner")
    parser.add_argument(
        "route", nargs="?", default=None,
        help="route string, e.g. LLFRLF (L=left R=right F=forward U=uturn "
             "S=stop). Omit to use ROUTE in route.py")
    args = parser.parse_args()
    route = parse_route(args.route) if args.route else list(ROUTE)

    px = Picarx()
    detector = AdaptiveLine(
        min_contrast=CONFIG.get("adaptive_min_contrast", 30),
        frac=CONFIG.get("adaptive_frac", 0.5),
    )

    period = 1.0 / FOLLOW_HZ
    mode = "follow"            # "follow" | "turn"
    turn_dir = 1
    turn_is_uturn = False
    last_steer = 0
    turn_start = 0.0
    cooldown_until = 0.0
    j = 0                      # next ROUTE index (also the junction count)

    print("Route runner: %d planned junctions %s. Ctrl+C to stop.\n"
          % (len(route), route))
    try:
        px.forward(DRIVE_POWER)
        while True:
            now = monotonic()
            data = px.get_grayscale_data()
            on_tape, signal, floor = detector.update(data)

            if mode == "follow":
                at_junction = on_tape[0] and on_tape[2]
                if at_junction and now >= cooldown_until:
                    action = route[j] if j < len(route) else "stop"
                    print("-> junction %d: %-8s data=%s on=%s floor=%.0f"
                          % (j, action, data, on_tape, floor))
                    j += 1

                    if action == "stop":
                        break
                    elif action == "straight":
                        px.set_dir_servo_angle(0)
                        px.forward(DRIVE_POWER)
                        last_steer = 0
                        cooldown_until = now + COOLDOWN_S   # carry across the crossbar
                    elif action in ("left", "right", "uturn"):
                        if CREEP_S > 0:                     # centre over the junction first
                            px.set_dir_servo_angle(0)
                            px.forward(DRIVE_POWER)
                            sleep(CREEP_S)
                        mode = "turn"
                        turn_is_uturn = (action == "uturn")
                        turn_dir = -1 if action == "right" else 1   # uturn pivots left
                        turn_start = monotonic()
                        pivot(px, turn_dir, PIVOT_POWER)
                    else:
                        print("   unknown action %r -> treating as straight" % action)
                        cooldown_until = now + COOLDOWN_S
                else:
                    steer = follow_steer(on_tape, last_steer)
                    last_steer = steer
                    px.set_dir_servo_angle(steer)
                    px.forward(DRIVE_POWER)

            elif mode == "turn":
                elapsed = now - turn_start
                min_s = MIN_UTURN_S if turn_is_uturn else MIN_TURN_S
                max_s = MAX_UTURN_S if turn_is_uturn else MAX_TURN_S
                centred = on_tape == [False, True, False]
                reacquired = elapsed >= min_s and centred
                if reacquired or elapsed >= max_s:
                    mode = "follow"
                    last_steer = 0
                    cooldown_until = monotonic() + COOLDOWN_S
                    px.set_dir_servo_angle(0)
                    px.forward(DRIVE_POWER)
                    print("   turn %s after %.1fs"
                          % ("complete" if reacquired else "TIMED OUT", elapsed))
                # else: keep pivoting (already commanded)

            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        px.stop()
        print("\nstopped after %d junction(s)" % j)


if __name__ == "__main__":
    main()
