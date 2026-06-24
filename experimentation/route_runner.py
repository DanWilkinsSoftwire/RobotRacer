#!/usr/bin/env python3
"""Route runner — follow the line, execute a PREDEFINED turn at each junction.

Detection (robust to this maze's varying / low-contrast surface):
    AdaptiveLine decides "on tape" from real-time contrast, anchored to the
    consistent tape level (~1000). A junction is detected when BOTH outer sensors
    read tape at once (a + / T crossbar -> [T,T,T]; a Y / fork -> [T,F,T]),
    debounced over a few frames. Normal following never lights both outer sensors.

Choice:
    ROUTE (route.py) or a route-string arg is an ordered list of actions, one
    consumed per junction (counted, no type checking).

Turns use a PIVOT (tank-turn) with the independent rear motors, closed-loop on
line reacquisition. Supports left/right/straight/uturn/stop.

Run it (claim the robot!):
    ./deploy.sh run experimentation/route_runner.py LLFRLF
    # L=left  R=right  F=forward(straight)  U=uturn  S=stop; omit to use route.py

Diagnose detection WITHOUT driving (hand-move the robot over the tape/junctions):
    ./deploy.sh run experimentation/route_runner.py --dry-run

Log a real run (per-cycle) to stdout and/or a CSV you can send back:
    ./deploy.sh run experimentation/route_runner.py FRLF --debug --log run.csv

No camera — pure grayscale, so it can run alongside grayscale_server.py.
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
CREEP_S = 0.15             # creep forward this long before pivoting, to centre the
                           # wheelbase over the junction. Set 0 to disable.
MIN_TURN_S = 0.4           # ignore line reacquisition before this (skip the crossbar we're on)
MAX_TURN_S = 2.5           # bail out of a 90-degree pivot after this
MIN_UTURN_S = 1.2          # a 180 takes longer; don't finish early
MAX_UTURN_S = 4.0
COOLDOWN_S = 1.0           # after handling a junction, ignore junctions briefly
SEED_S = 0.5               # sit still this long at startup to seed the floor estimate
JUNCTION_FRAMES = 3        # require both-outer-on-tape for this many consecutive
                           # cycles before acting — debounces single-frame glitches
LOG_EVERY = 5              # with --debug, log every Nth control cycle (~10 Hz at 50 Hz)

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

    Case-insensitive; spaces, commas, dashes and underscores are ignored.
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


def bits(on_tape):
    return "".join("T" if b else "F" for b in on_tape)


class Logger:
    """Print log lines and (optionally) tee them to a file."""

    def __init__(self, path=None):
        self.f = open(path, "w") if path else None
        if self.f:
            self.f.write("# t,data,floor,thr,on,mode,steer,junction_frames,note\n")

    def line(self, msg):
        print(msg, flush=True)

    def row(self, t, data, floor, thr, on_tape, mode, steer, jf, note=""):
        msg = ("[%6.2f] data=%s floor=%4.0f thr=%4.0f on=%s mode=%-6s steer=%+d jf=%d %s"
               % (t, data, floor, thr, bits(on_tape), mode, steer, jf, note))
        print(msg, flush=True)
        if self.f:
            self.f.write("%.3f,%s,%.0f,%.0f,%s,%s,%d,%d,%s\n"
                         % (t, "|".join(map(str, data)), floor, thr,
                            bits(on_tape), mode, steer, jf, note))
            self.f.flush()

    def close(self):
        if self.f:
            self.f.close()


class Motors:
    """Thin actuation wrapper so --dry-run can disable all movement."""

    def __init__(self, px, enabled):
        self.px = px
        self.enabled = enabled

    def forward(self, speed):
        if self.enabled:
            self.px.forward(speed)

    def stop(self):
        if self.enabled:
            self.px.stop()

    def set_dir_servo_angle(self, angle):
        if self.enabled:
            self.px.set_dir_servo_angle(angle)

    def set_motor_speed(self, motor, speed):
        if self.enabled:
            self.px.set_motor_speed(motor, speed)


def pivot(mot, direction, power):
    """direction: +1 = left (CCW), -1 = right (CW). Rear-wheel tank-turn in place.

    Signs follow picarx forward() = (motor1:+, motor2:-):
        left  (CCW) = both motors negative
        right (CW)  = both motors positive
    If your robot pivots the WRONG way, swap the two motor signs below.
    """
    mot.set_dir_servo_angle(direction * TURN_ANGLE)   # front wheels into the turn
    if direction > 0:        # left
        mot.set_motor_speed(1, -power)
        mot.set_motor_speed(2, -power)
    else:                    # right
        mot.set_motor_speed(1, power)
        mot.set_motor_speed(2, power)


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


def seed_floor(px, detector, log):
    log.line("Seeding floor (hold still)...")
    seed = []
    t0 = monotonic()
    while monotonic() - t0 < SEED_S:
        seed.append(px.get_grayscale_data())
        sleep(0.02)
    detector.seed_floor(seed)
    log.line("  samples=%s" % seed[:3] + (" ..." if len(seed) > 3 else ""))
    log.line("  floor=%s  threshold=%.0f  (tape~%.0f)"
             % (None if detector.floor is None else round(detector.floor),
                detector.threshold(), detector.tape_level))


def dry_run(px, detector, log):
    """Read + log detection forever, no motors. Hand-move the robot to diagnose."""
    log.line("\nDRY RUN: no motors. Move the robot over tape / junctions; Ctrl+C to stop.\n")
    t0 = monotonic()
    last = None
    try:
        while True:
            data = px.get_grayscale_data()
            on_tape, thr, floor = detector.update(data)
            junction = on_tape[0] and on_tape[2]
            note = "JUNCTION" if junction else ("LOST" if not any(on_tape) else "")
            # log every cycle on change, plus a heartbeat
            if on_tape != last:
                log.row(monotonic() - t0, data, floor, thr, on_tape, "dry", 0, 0, note)
                last = on_tape
            sleep(0.05)
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(
        description="PiCar-X predefined-route maze runner")
    parser.add_argument(
        "route", nargs="?", default=None,
        help="route string, e.g. LLFRLF (L=left R=right F=forward U=uturn "
             "S=stop). Omit to use ROUTE in route.py")
    parser.add_argument("--dry-run", action="store_true",
                        help="read + log sensors only, no movement (hand-move to diagnose)")
    parser.add_argument("--debug", action="store_true",
                        help="log every control cycle")
    parser.add_argument("--log", metavar="PATH", default=None,
                        help="also write per-cycle CSV to this file")
    args = parser.parse_args()

    route = parse_route(args.route) if args.route else list(ROUTE)
    log = Logger(args.log)

    px = Picarx()
    detector = AdaptiveLine(
        tape_level=CONFIG.get("tape_level", 1000),
        frac=CONFIG.get("adaptive_frac", 0.5),
        min_contrast=CONFIG.get("adaptive_min_contrast", 40),
    )
    log.line("config: drive_power=%s steer_offset=%s tape_level=%s frac=%s min_contrast=%s"
             % (DRIVE_POWER, STEER_OFFSET, detector.tape_level, detector.frac,
                detector.min_contrast))

    seed_floor(px, detector, log)

    if args.dry_run:
        dry_run(px, detector, log)
        log.close()
        return

    mot = Motors(px, enabled=True)
    period = 1.0 / FOLLOW_HZ
    mode = "follow"            # "follow" | "turn"
    turn_dir = 1
    turn_is_uturn = False
    last_steer = 0
    turn_start = 0.0
    cooldown_until = 0.0
    junction_frames = 0       # consecutive both-outer-on-tape cycles (debounce)
    j = 0                     # next ROUTE index (also the junction count)
    cycle = 0
    t_start = monotonic()

    log.line("Route runner: %d planned junctions %s. Ctrl+C to stop.\n"
             % (len(route), route))
    try:
        mot.forward(DRIVE_POWER)
        while True:
            now = monotonic()
            cycle += 1
            data = px.get_grayscale_data()
            on_tape, thr, floor = detector.update(data)

            if mode == "follow":
                junction_frames = junction_frames + 1 if (on_tape[0] and on_tape[2]) else 0
                if junction_frames >= JUNCTION_FRAMES and now >= cooldown_until:
                    junction_frames = 0
                    action = route[j] if j < len(route) else "stop"
                    log.line("-> junction %d: %-8s data=%s on=%s thr=%.0f floor=%.0f"
                             % (j, action, data, bits(on_tape), thr, floor))
                    j += 1

                    if action == "stop":
                        break
                    elif action == "straight":
                        mot.set_dir_servo_angle(0)
                        mot.forward(DRIVE_POWER)
                        last_steer = 0
                        cooldown_until = now + COOLDOWN_S   # carry across the crossbar
                    elif action in ("left", "right", "uturn"):
                        if CREEP_S > 0:                     # centre over the junction first
                            mot.set_dir_servo_angle(0)
                            mot.forward(DRIVE_POWER)
                            sleep(CREEP_S)
                        mode = "turn"
                        turn_is_uturn = (action == "uturn")
                        turn_dir = -1 if action == "right" else 1   # uturn pivots left
                        turn_start = monotonic()
                        pivot(mot, turn_dir, PIVOT_POWER)
                    else:
                        log.line("   unknown action %r -> treating as straight" % action)
                        cooldown_until = now + COOLDOWN_S
                else:
                    steer = follow_steer(on_tape, last_steer)
                    last_steer = steer
                    mot.set_dir_servo_angle(steer)
                    mot.forward(DRIVE_POWER)

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
                    mot.set_dir_servo_angle(0)
                    mot.forward(DRIVE_POWER)
                    log.line("   turn %s after %.1fs"
                             % ("complete" if reacquired else "TIMED OUT", elapsed))
                # else: keep pivoting (already commanded)

            if args.debug and cycle % LOG_EVERY == 0:
                log.row(now - t_start, data, floor, thr, on_tape, mode,
                        last_steer, junction_frames)

            sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        mot.stop()
        log.line("\nstopped after %d junction(s)" % j)
        log.close()


if __name__ == "__main__":
    main()
