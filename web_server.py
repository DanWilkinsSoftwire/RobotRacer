#!/usr/bin/env python3
"""Camera web server + video recording + live grayscale debug dashboard.

Two web endpoints run side by side:

  * Camera MJPEG stream (served by Vilib):
        http://<robot-ip>:9000/mjpg
  * Debug dashboard (served by us, embeds the camera + live sensors):
        http://<robot-ip>:9001/

The dashboard shows the three grayscale sensor readings in real time with the
config.json line reference marked on each bar, so you can see exactly what the
sensors read over the white tape vs the dark floor while tuning the maze.

Camera/recording follows the SunFounder record tutorial:
https://docs.sunfounder.com/projects/picar-x-v20/en/latest/python/python_record.html

Keyboard controls (in the terminal running this script):
    Q: record / pause / continue
    E: stop (saves the .avi)
    Ctrl + C: quit
"""

from time import sleep, strftime, localtime
from threading import Thread
import os

from vilib import Vilib
from picarx import Picarx
import readchar
from flask import Flask, jsonify, Response

try:
    from config import CONFIG
    LINE_REFERENCE = CONFIG.get("line_reference", [1400, 1400, 1400])
except Exception:
    LINE_REFERENCE = [1400, 1400, 1400]

# robot_hat ADC is 12-bit, so grayscale readings range 0..4095 (brighter -> higher).
ADC_MAX = 4095
DEBUG_PORT = 9001
CAM_PORT = 9000  # Vilib's MJPEG port

px = Picarx()
px.set_line_reference(LINE_REFERENCE)

manual = '''
Live camera:      http://<robot-ip>:%d/mjpg
Debug dashboard:  http://<robot-ip>:%d/

Press keys to control recording:
    Q: record/pause/continue
    E: stop
    Ctrl + C: quit
''' % (CAM_PORT, DEBUG_PORT)


# ----------------------------------------------------------------------------
# Debug web server (Flask, runs in a background thread)
# ----------------------------------------------------------------------------
app = Flask(__name__)


def read_sensors():
    """Return current sensor state, swallowing transient read errors."""
    try:
        data = px.get_grayscale_data()  # [A0, A1, A2]
    except Exception as e:
        return {"error": str(e)}

    # The library's interpretation. NOTE: get_line_status assumes a DARK line on
    # a LIGHT background, which is inverted from our white-tape-on-dark-floor
    # maze, so trust the raw values + "over reference" more than this label.
    try:
        line_status = px.get_line_status(data)  # [l, m, r], 0=line 1=background
    except Exception:
        line_status = None

    try:
        distance = round(px.get_distance(), 1)
    except Exception:
        distance = None

    return {
        "data": list(data),
        "reference": LINE_REFERENCE,
        # True where the sensor reads brighter than its threshold (i.e. white tape)
        "over_reference": [int(v) > int(r) for v, r in zip(data, LINE_REFERENCE)],
        "line_status": line_status,
        "distance": distance,
        "adc_max": ADC_MAX,
    }


@app.route("/grayscale")
def grayscale():
    return jsonify(read_sensors())


@app.route("/")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


def start_debug_server():
    # threaded=True so polling the JSON endpoint doesn't block; no reloader in a thread.
    app.run(host="0.0.0.0", port=DEBUG_PORT, threaded=True,
            debug=False, use_reloader=False)


DASHBOARD_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport"
  content="width=device-width, initial-scale=1">
<title>PiCar grayscale debug</title>
<style>
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         margin:0; padding:16px; }
  h1 { font-size:1.1rem; font-weight:600; margin:0 0 12px; }
  .wrap { display:flex; flex-wrap:wrap; gap:20px; align-items:flex-start; }
  img.cam { max-width:480px; width:100%; border-radius:8px; background:#000; }
  .sensors { flex:1; min-width:280px; }
  .row { display:flex; align-items:center; gap:10px; margin:10px 0; }
  .label { width:70px; font-variant-numeric:tabular-nums; }
  .bar { position:relative; flex:1; height:26px; background:#222;
         border-radius:4px; overflow:hidden; }
  .fill { height:100%; transition:width .08s linear; }
  .fill.over { background:#39d353; }      /* over threshold = bright/white tape */
  .fill.under { background:#555; }        /* under threshold = dark floor */
  .ref { position:absolute; top:0; bottom:0; width:2px; background:#ff5277; }
  .val { width:60px; text-align:right; font-variant-numeric:tabular-nums; }
  .meta { margin-top:16px; color:#aaa; font-size:.9rem; line-height:1.6; }
  .meta b { color:#eee; }
  code { color:#9cdcfe; }
</style></head>
<body>
  <h1>PiCar-X grayscale debug</h1>
  <div class="wrap">
    <img class="cam" id="cam" alt="camera stream">
    <div class="sensors">
      <div class="row"><div class="label">Left A0</div>
        <div class="bar"><div class="fill" id="f0"></div><div class="ref" id="r0"></div></div>
        <div class="val" id="v0">--</div></div>
      <div class="row"><div class="label">Mid A1</div>
        <div class="bar"><div class="fill" id="f1"></div><div class="ref" id="r1"></div></div>
        <div class="val" id="v1">--</div></div>
      <div class="row"><div class="label">Right A2</div>
        <div class="bar"><div class="fill" id="f2"></div><div class="ref" id="r2"></div></div>
        <div class="val" id="v2">--</div></div>
      <div class="meta">
        <div>Reference: <code id="ref">--</code>
          (green = over threshold &rarr; white tape)</div>
        <div>Library line_status: <code id="ls">--</code></div>
        <div>Distance: <b id="dist">--</b> cm</div>
        <div id="err" style="color:#ff5277"></div>
      </div>
    </div>
  </div>
<script>
  // Camera stream lives on a different port on this same host.
  document.getElementById('cam').src =
      'http://' + location.hostname + ':%CAM_PORT%/mjpg';

  const ADC_MAX = %ADC_MAX%;
  async function poll() {
    try {
      const r = await fetch('/grayscale', {cache:'no-store'});
      const d = await r.json();
      const err = document.getElementById('err');
      if (d.error) { err.textContent = 'sensor error: ' + d.error; return; }
      err.textContent = '';
      for (let i = 0; i < 3; i++) {
        const v = d.data[i], ref = d.reference[i], over = d.over_reference[i];
        const fill = document.getElementById('f' + i);
        fill.style.width = Math.min(100, 100 * v / ADC_MAX) + '%';
        fill.className = 'fill ' + (over ? 'over' : 'under');
        document.getElementById('r' + i).style.left =
            (100 * ref / ADC_MAX) + '%';
        document.getElementById('v' + i).textContent = v;
      }
      document.getElementById('ref').textContent = '[' + d.reference.join(', ') + ']';
      document.getElementById('ls').textContent =
          d.line_status ? JSON.stringify(d.line_status) : 'n/a';
      document.getElementById('dist').textContent =
          (d.distance === null ? '--' : d.distance);
    } catch (e) { /* keep polling */ }
  }
  setInterval(poll, 120);
  poll();
</script>
</body></html>
""".replace("%CAM_PORT%", str(CAM_PORT)).replace("%ADC_MAX%", str(ADC_MAX))


def print_overwrite(msg, end='', flush=True):
    print('\r\033[2K', end='', flush=True)
    print(msg, end=end, flush=True)


def main():
    rec_flag = 'stop'  # start, pause, stop
    vname = None
    username = os.getlogin()

    Vilib.rec_video_set["path"] = f"/home/{username}/Videos/"  # save path

    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)  # web=True serves the MJPEG stream
    sleep(0.8)  # wait for startup

    # Start the debug dashboard alongside the camera stream.
    Thread(target=start_debug_server, daemon=True).start()

    print(manual)
    while True:
        key = readchar.readkey().lower()
        # record / pause / continue
        if key == 'q':
            if rec_flag == 'stop':
                rec_flag = 'start'
                vname = strftime("%Y-%m-%d-%H.%M.%S", localtime())
                Vilib.rec_video_set["name"] = vname
                Vilib.rec_video_run()
                Vilib.rec_video_start()
                print_overwrite('rec start ...')
            elif rec_flag == 'start':
                rec_flag = 'pause'
                Vilib.rec_video_pause()
                print_overwrite('pause')
            elif rec_flag == 'pause':
                rec_flag = 'start'
                Vilib.rec_video_start()
                print_overwrite('continue')
        # stop
        elif key == 'e' and rec_flag != 'stop':
            rec_flag = 'stop'
            Vilib.rec_video_stop()
            print_overwrite("The video saved as %s%s.avi" % (
                Vilib.rec_video_set["path"], vname), end='\n')
        # quit
        elif key == readchar.key.CTRL_C:
            Vilib.camera_close()
            print('\nquit')
            break

        sleep(0.1)


if __name__ == "__main__":
    main()
