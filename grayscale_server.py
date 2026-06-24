#!/usr/bin/env python3
"""Standalone live grayscale debug dashboard.

Reads the three grayscale sensors over I2C (robot_hat ADC) and serves a small
web page showing them in real time. Deliberately has NO camera and NO Picarx:

  * no camera  -> never fights web_server.py / Vilib over the camera
  * no Picarx  -> claims no motor/servo GPIO, so it can run *alongside* the
                  line-follower while it drives the maze ("GPIO busy"-proof)

Run it:
    python3 grayscale_server.py
Then open from any device on the network:
    http://<robot-ip>:9001/        e.g. http://champ4:9001/

The bars are scaled to the 12-bit ADC range and the config.json line_reference
is marked on each; a bar turns green when the sensor reads brighter than its
threshold (i.e. it's over the white tape).
"""

from flask import Flask, jsonify, Response
from robot_hat import ADC

try:
    from config import CONFIG
    LINE_REFERENCE = CONFIG.get("line_reference", [1400, 1400, 1400])
except Exception:
    LINE_REFERENCE = [1400, 1400, 1400]

ADC_MAX = 4095          # robot_hat ADC is 12-bit; brighter surface -> higher value
PORT = 9001
GRAYSCALE_PINS = ["A0", "A1", "A2"]   # Picarx grayscale defaults (left, mid, right)
SENSOR_LABELS = ["Left", "Mid", "Right"]

adcs = [ADC(pin) for pin in GRAYSCALE_PINS]

app = Flask(__name__)


def read_sensors():
    """Return current sensor state, swallowing transient read errors."""
    try:
        data = [adc.read() for adc in adcs]  # [A0, A1, A2] = left, mid, right
    except Exception as e:
        return {"error": str(e)}

    # White tape reads brighter than the dark floor, so "over reference" == tape.
    over = [int(v) > int(r) for v, r in zip(data, LINE_REFERENCE)]
    tape = [SENSOR_LABELS[i] for i, o in enumerate(over) if o]

    return {
        "data": data,
        "reference": LINE_REFERENCE,
        "over_reference": over,
        "tape": tape,          # which sensors currently see white tape
        "adc_max": ADC_MAX,
    }


@app.route("/grayscale")
def grayscale():
    return jsonify(read_sensors())


@app.route("/")
def dashboard():
    return Response(DASHBOARD_HTML, mimetype="text/html")


DASHBOARD_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport"
  content="width=device-width, initial-scale=1">
<title>PiCar grayscale debug</title>
<style>
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         margin:0; padding:24px; max-width:560px; }
  h1 { font-size:1.1rem; font-weight:600; margin:0 0 16px; }
  .row { display:flex; align-items:center; gap:10px; margin:12px 0; }
  .label { width:60px; font-variant-numeric:tabular-nums; }
  .bar { position:relative; flex:1; height:30px; background:#222;
         border-radius:4px; overflow:hidden; }
  .fill { height:100%; transition:width .08s linear; }
  .fill.over { background:#39d353; }      /* over threshold = bright/white tape */
  .fill.under { background:#555; }        /* under threshold = dark floor */
  .ref { position:absolute; top:0; bottom:0; width:2px; background:#ff5277; }
  .val { width:60px; text-align:right; font-variant-numeric:tabular-nums; }
  .meta { margin-top:20px; color:#aaa; font-size:.9rem; line-height:1.7; }
  .meta b { color:#eee; }
  code { color:#9cdcfe; }
  button { background:#333; color:#eee; border:1px solid #555; border-radius:4px;
           padding:2px 8px; cursor:pointer; font-size:.85rem; }
</style></head>
<body>
  <h1>PiCar-X grayscale debug</h1>
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
    <div>Tape detected under: <b id="tape">--</b></div>
    <div>Bar scale (auto): <code id="scale">--</code>
      <button id="reset">reset</button></div>
    <div id="err" style="color:#ff5277"></div>
  </div>
<script>
  // Auto-scale: stretch the bars to the range of values + references we've seen
  // (with a little padding) so threshold crossings are big, obvious movements.
  let lo = null, hi = null;
  document.getElementById('reset').onclick = () => { lo = hi = null; };

  async function poll() {
    try {
      const r = await fetch('/grayscale', {cache:'no-store'});
      const d = await r.json();
      const err = document.getElementById('err');
      if (d.error) { err.textContent = 'sensor error: ' + d.error; return; }
      err.textContent = '';

      const pts = d.data.concat(d.reference);
      lo = (lo === null) ? Math.min(...pts) : Math.min(lo, ...pts);
      hi = (hi === null) ? Math.max(...pts) : Math.max(hi, ...pts);
      const pad = Math.max(1, (hi - lo) * 0.05);
      const LO = lo - pad, HI = hi + pad;
      const scale = v => Math.max(0, Math.min(100, 100 * (v - LO) / (HI - LO)));

      for (let i = 0; i < 3; i++) {
        const v = d.data[i], ref = d.reference[i], over = d.over_reference[i];
        const fill = document.getElementById('f' + i);
        fill.style.width = scale(v) + '%';
        fill.className = 'fill ' + (over ? 'over' : 'under');
        document.getElementById('r' + i).style.left = scale(ref) + '%';
        document.getElementById('v' + i).textContent = v;
      }
      document.getElementById('ref').textContent = '[' + d.reference.join(', ') + ']';
      document.getElementById('tape').textContent =
          d.tape.length ? d.tape.join(', ') : 'none';
      document.getElementById('scale').textContent =
          Math.round(LO) + ' – ' + Math.round(HI);
    } catch (e) { /* keep polling */ }
  }
  setInterval(poll, 120);
  poll();
</script>
</body></html>
"""


if __name__ == "__main__":
    print("grayscale dashboard: http://<robot-ip>:%d/" % PORT)
    app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False, use_reloader=False)
