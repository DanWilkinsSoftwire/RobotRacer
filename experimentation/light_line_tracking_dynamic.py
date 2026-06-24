'''
    Light Line Following for Picar-X — with LIVE tuning via config.json.

    Same line-following logic as light_line_tracking.py (white/light line on a
    dark background), but `drive_power` (speed) and `steer_offset` (turning) are
    re-read from config.json whenever the file changes, instead of being baked in
    as constants. Run experimentation/tuning_server.py in another terminal to drag
    sliders and feel the change immediately — no edit / commit / redeploy cycle to
    find the speed limit.

    Run BOTH on the robot, in two terminals:
        ./deploy.sh run experimentation/tuning_server.py            # the web UI
        ./deploy.sh run experimentation/light_line_tracking_dynamic.py   # the follower
    Tuning UI: http://<robot-ip>:9002/   (e.g. http://champ4:9002/)
    Ctrl + C to stop. Motors stop on exit.

    Grayscale reference: set in config.json (line_reference), same as before.
'''
import os
import sys

# Repo root on the path so `import config` works when run from experimentation/.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from time import sleep

from picarx import Picarx

import config

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")

px = Picarx()
# px.set_line_reference([1400, 1400, 1400])  # or auto-calibrate; see docs/picar-api.md

current_state = None
last_state = "stop"
RECOVERY_ANGLE = 30   # full lock while backing up to re-find a lost line

# Live config: re-read config.json only when it changes (mtime), so the hot loop
# isn't re-parsing a file every iteration. tuning_server.py writes it atomically.
_cfg = config.load()
_cfg_mtime = os.stat(CONFIG_PATH).st_mtime_ns


def live_config():
    global _cfg, _cfg_mtime
    m = os.stat(CONFIG_PATH).st_mtime_ns
    if m != _cfg_mtime:
        _cfg_mtime = m
        _cfg = config.load()
    return _cfg


def outHandle():
    global last_state, current_state
    power = live_config()["drive_power"]
    if last_state == 'left':
        px.set_dir_servo_angle(-RECOVERY_ANGLE)
        px.backward(power)
    elif last_state == 'right':
        px.set_dir_servo_angle(RECOVERY_ANGLE)
        px.backward(power)
    while True:
        gm_val_list = px.get_grayscale_data()
        gm_state = get_status(gm_val_list)
        print("outHandle gm_val_list: %s, %s"%(gm_val_list, gm_state))
        currentSta = gm_state
        if currentSta != last_state:
            break
    sleep(0.001)


def get_status(val_list):
    _state = px.get_line_status(val_list)  # [bool, bool, bool], 0 means white, 1 means black

    # Invert the state list to handle white line on dark background
    _state = [1 - x for x in _state]       # Now: 1 represents the white line, 0 represents background

    if _state == [0, 0, 0]:
        return 'stop'
    elif _state[1] == 1:
        return 'forward'
    elif _state[0] == 1:
        return 'right'
    elif _state[2] == 1:
        return 'left'


if __name__=='__main__':
    print("Tuning UI: http://<robot-ip>:9002/  (run experimentation/tuning_server.py)\n")
    try:
        while True:
            cfg = live_config()              # picks up slider changes
            power = cfg["drive_power"]
            offset = cfg["steer_offset"]
            # Corner speed: full power on straights, slowed while correcting.
            # Mirrors the PID follower's speed = base * (1 - speed_scale*error),
            # with error = 0 (centred) or 1 (off to a side) for bang-bang.
            corner_power = max(1, round(power * (1.0 - cfg["speed_scale"])))

            gm_val_list = px.get_grayscale_data()
            gm_state = get_status(gm_val_list)
            print("gm_val_list: %s, %s"%(gm_val_list, gm_state))

            if gm_state != "stop":
                last_state = gm_state

            if gm_state == 'forward':
                px.set_dir_servo_angle(0)
                px.forward(power)
            elif gm_state == 'left':
                px.set_dir_servo_angle(offset)
                px.forward(corner_power)
            elif gm_state == 'right':
                px.set_dir_servo_angle(-offset)
                px.forward(corner_power)
            else:
                outHandle()

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stop and exit")

    finally:
        px.stop()
        print("stop and exit")
        sleep(0.1)
