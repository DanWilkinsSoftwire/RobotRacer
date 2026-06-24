"""Loads calibration / tuning from config.json.

One robot, one floor: config.json is committed and shared. Tune the values,
commit, push, and everyone gets the dialed-in numbers via `git pull`.

    from config import CONFIG
    px.set_line_reference(CONFIG["line_reference"])
"""
import json
import os

_HERE = os.path.dirname(os.path.abspath(__file__))


def load():
    with open(os.path.join(_HERE, "config.json")) as f:
        data = json.load(f)
    data.pop("_comment", None)
    return data


CONFIG = load()
