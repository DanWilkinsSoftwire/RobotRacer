# PiCar-X API Reference (for maze / line-following)

> Assumes the **SunFounder PiCar-X** (the variant with the 3-sensor grayscale
> module used for line tracking). If you actually have a PiCar-V (camera-only)
> or PiCar-S, the API differs — tell Claude and we'll swap this out.

## Install / setup

The library is `picarx`, built on `robot_hat`. On the Pi it's usually
pre-installed under `/opt/picar-x/`. Config (calibration) lives in
`/opt/picar-x/picar-x.conf`.

```python
from picarx import Picarx
px = Picarx()
```

## Core API — `Picarx` class

### Movement
| Method | Description |
|---|---|
| `forward(speed)` | Drive forward. `speed` ~0–100. Applies differential steering based on current servo angle. |
| `backward(speed)` | Drive backward. |
| `stop()` | Stop both motors (called twice internally for reliability). |
| `set_power(speed)` | Set both motors to the same raw speed. |
| `reset()` | Stop motors + return all servos to neutral. |

### Steering (front wheels)
| Method | Range | Description |
|---|---|---|
| `set_dir_servo_angle(value)` | **[-30, 30]** | Steering angle in degrees. 0 = straight, negative = left/right per calibration. |

### Camera servos
| Method | Range |
|---|---|
| `set_cam_pan_angle(value)` | [-90, 90] |
| `set_cam_tilt_angle(value)` | [-35, 65] |

### Grayscale / line sensing (the important bit for the maze)
| Method | Description |
|---|---|
| `get_grayscale_data()` | Returns `[left, mid, right]` raw readings. **Brighter surface → larger value.** So white tape reads HIGH, dark floor reads LOW. |
| `get_line_status(gm_val_list)` | Returns a 3-element list like `[0,1,0]`. Per SunFounder's example: `0` = line under that sensor, `1` = background. (Verify on your floor — see note below.) |
| `set_line_reference([l, m, r])` | Set the 3 per-sensor thresholds that separate line vs background. |
| `set_grayscale_reference(value)` | Alias for `set_line_reference`. |

### Cliff detection (handy to avoid driving off a table edge)
| Method | Description |
|---|---|
| `get_cliff_status(gm_val_list)` | `True` if any sensor reads below the cliff threshold. |
| `set_cliff_reference([l, m, r])` | Set cliff thresholds. |

### Ultrasonic (wall distance — useful for maze walls, if present)
| Method | Description |
|---|---|
| `get_distance()` | Distance in cm from the front ultrasonic sensor. |

### Calibration helpers (run once, values persist to config)
`dir_servo_calibrate(value)`, `cam_pan_servo_calibrate(value)`,
`cam_tilt_servo_calibrate(value)`, `motor_direction_calibrate(motor, value)`,
`motor_speed_calibration(value)`.

### Constructor defaults
```python
Picarx(
    servo_pins=['P0','P1','P2'],
    motor_pins=['D4','D5','P13','P12'],
    grayscale_pins=['A0','A1','A2'],
    ultrasonic_pins=['D2','D3'],
    config='/opt/picar-x/picar-x.conf',
)
```

## Setting the grayscale reference

Two options:
1. **Auto:** run `./calibration/grayscale_calibration.py` from the picar-x repo
   on your actual maze surface.
2. **Manual:** `px.set_line_reference([1400, 1400, 1400])` — pick a value between
   a typical "on white tape" reading and a typical "on dark floor" reading.

> ⚠️ **White tape on dark floor is inverted vs the stock demo**, which assumes a
> *dark line on a light floor*. The stock `get_line_status` semantics (0 = line)
> may come out backwards for you. Easiest robust approach: read
> `get_grayscale_data()` directly and write your own logic — "highest of the 3
> sensors = where the white tape is." Drive a sensor-survey first
> (print readings on tape vs floor) to fix your threshold.

## Minimal line-follow loop (SunFounder stock example)

```python
from picarx import Picarx
from time import sleep

px = Picarx()
# px.set_line_reference([1400, 1400, 1400])  # or auto-calibrate

px_power = 10
offset = 20
last_state = "stop"

def get_status(val_list):
    state = px.get_line_status(val_list)   # [l, m, r]; 0 = line, 1 = background
    if state == [0, 0, 0]:
        return 'stop'
    elif state[1] == 1:
        return 'forward'
    elif state[0] == 1:
        return 'right'
    elif state[2] == 1:
        return 'left'

try:
    while True:
        val = px.get_grayscale_data()
        st = get_status(val)
        if st != "stop":
            last_state = st
        if st == 'forward':
            px.set_dir_servo_angle(0);       px.forward(px_power)
        elif st == 'left':
            px.set_dir_servo_angle(offset);  px.forward(px_power)
        elif st == 'right':
            px.set_dir_servo_angle(-offset); px.forward(px_power)
        # else: handle lost-line recovery (back up toward last_state)
except KeyboardInterrupt:
    pass
finally:
    px.stop()
    sleep(0.1)
```

## Sources
- Line Tracking guide: https://docs.sunfounder.com/projects/picar-x-v20/en/latest/python/python_line_track.html
- Full docs: https://docs.sunfounder.com/projects/picar-x/en/stable/
- Source (`picarx.py`): https://github.com/sunfounder/picar-x/blob/v2.0/picarx/picarx.py
- Code reference (DeepWiki): https://deepwiki.com/sunfounder/picar-x/8-reference
