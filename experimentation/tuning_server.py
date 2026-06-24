#!/usr/bin/env python3
"""Standalone live-tuning web UI for the line follower.

Separate process on purpose (same idea as grayscale_server.py): it creates NO
Picarx, so it claims no motor/servo GPIO and can run *alongside*
light_line_tracking_dynamic.py without "GPIO busy" clashes.

A separate process can't share Python memory with the follower, so the channel
between them is config.json on disk: every slider move writes the new value to
config.json (atomically, via a temp file + os.replace), and the follower reloads
it live. Because each change lands straight in config.json, there's no separate
"save" step — whatever you leave the sliders on is what `git add config.json`
will commit.

Run it on the robot (its own terminal / ssh session):
    python3 experimentation/tuning_server.py
Open from any device on the network:
    http://<robot-ip>:9002/        e.g. http://champ4:9002/
"""

import json
import os
import threading

from flask import Flask, Response, jsonify, request

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(_ROOT, "config.json")
PORT = 9002

# Slider metadata. `key` must match a config.json key. The follower only reads
# drive_power (speed) and steer_offset (turning), so those are the only knobs.
PARAMS = [
    {"key": "drive_power",  "label": "Drive power",        "min": 0, "max": 100, "step": 1, "cast": "int", "group": "Speed"},
    {"key": "steer_offset", "label": "Steer offset (deg)", "min": 0, "max": 30,  "step": 1, "cast": "int", "group": "Turning"},
]
_SPEC = {p["key"]: p for p in PARAMS}
_CAST = {"int": int, "float": float}

# Serialise read-modify-write of config.json against concurrent slider POSTs.
_lock = threading.Lock()

app = Flask(__name__)


def read_config():
    with open(CONFIG_PATH) as f:
        return json.load(f)


def current_values():
    data = read_config()
    return {p["key"]: _CAST[p["cast"]](data[p["key"]]) for p in PARAMS}


def write_param(key, value):
    """Cast + clamp to the param's range, write into config.json atomically."""
    spec = _SPEC[key]  # KeyError for unknown key -> 400 in the route
    v = _CAST[spec["cast"]](value)
    v = max(spec["min"], min(spec["max"], v))
    with _lock:
        data = read_config()
        data[key] = v
        tmp = CONFIG_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        os.replace(tmp, CONFIG_PATH)   # atomic: follower never sees a half-written file
    return v


@app.route("/params")
def params():
    return jsonify({"params": PARAMS, "values": current_values()})


@app.route("/params", methods=["POST"])
def set_param():
    body = request.get_json(force=True)
    try:
        value = write_param(body["key"], body["value"])
    except KeyError:
        return jsonify({"error": "unknown param %r" % body.get("key")}), 400
    return jsonify({"key": body["key"], "value": value})


@app.route("/")
def index():
    return Response(PAGE_HTML, mimetype="text/html")


PAGE_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport"
  content="width=device-width, initial-scale=1">
<title>PiCar tuning</title>
<style>
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         margin:0; padding:24px; max-width:560px; }
  h1 { font-size:1.1rem; font-weight:600; margin:0 0 4px; }
  h2 { font-size:.8rem; font-weight:600; text-transform:uppercase;
       letter-spacing:.05em; color:#9cdcfe; margin:22px 0 8px; }
  .row { display:flex; align-items:center; gap:12px; margin:10px 0; }
  .label { flex:1; }
  input[type=range] { flex:2; accent-color:#39d353; }
  .val { width:54px; text-align:right; font-variant-numeric:tabular-nums;
         color:#39d353; }
  #status { color:#aaa; font-size:.85rem; }
</style></head>
<body>
  <h1>PiCar-X tuning</h1>
  <div id="status">loading...</div>
  <div id="groups"></div>
<script>
  let SPEC = [];

  function fmt(p, v) { return p.cast === 'int' ? v : Number(v).toFixed(1); }

  async function post(key, value) {
    const r = await fetch('/params', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({key, value})});
    return (await r.json()).value;
  }

  function render(values) {
    const groups = {};
    SPEC.forEach(p => { (groups[p.group] = groups[p.group] || []).push(p); });
    const root = document.getElementById('groups');
    root.innerHTML = '';
    for (const [name, params] of Object.entries(groups)) {
      const h = document.createElement('h2'); h.textContent = name; root.appendChild(h);
      params.forEach(p => {
        const row = document.createElement('div'); row.className = 'row';
        const label = document.createElement('span'); label.className = 'label';
        label.textContent = p.label;
        const slider = document.createElement('input');
        slider.type = 'range'; slider.min = p.min; slider.max = p.max;
        slider.step = p.step; slider.value = values[p.key];
        const out = document.createElement('span'); out.className = 'val';
        out.textContent = fmt(p, values[p.key]);
        slider.oninput = async () => {
          out.textContent = fmt(p, slider.value);
          const v = await post(p.key, parseFloat(slider.value));
          slider.value = v; out.textContent = fmt(p, v);
        };
        row.append(label, slider, out); root.appendChild(row);
      });
    }
  }

  async function load() {
    const r = await fetch('/params', {cache:'no-store'});
    const d = await r.json();
    SPEC = d.params;
    render(d.values);
    document.getElementById('status').textContent =
        'Live — each change is written to config.json and the follower picks it up.';
  }

  load();
</script>
</body></html>
"""


if __name__ == "__main__":
    print("tuning UI: http://<robot-ip>:%d/" % PORT)
    app.run(host="0.0.0.0", port=PORT, threaded=True, debug=False, use_reloader=False)
