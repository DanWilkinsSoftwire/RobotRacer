#!/usr/bin/env python3
"""Camera-based junction detector — STEP 1: detection only, no motors.

Architecture (the shape the maze solver will grow into):
    - A *vision worker* thread grabs frames from Vilib, thresholds them, and
      classifies what's ahead into a shared `JunctionState`.
    - The *main thread* is the consumer. For now it just prints the state so we
      can verify detection by hand-driving the robot. Later, the fast
      grayscale+motor control loop replaces this consumer and reads the same
      shared state to decide turns — vision never blocks control.

How detection works (deliberately simple + heavily instrumented):
    We threshold the frame (white tape = bright = foreground; we're inverted
    vs the stock dark-line demo) and look at two horizontal bands:
        NEAR band  — low in the frame, ~ where the robot is now.
        FAR  band  — high in the frame, the look-ahead.
    In each band we collapse to a 1-D column profile and find white "segments".
    Segment count + whether tape touches the left/right edge gives a coarse
    label. The raw numbers are printed too — TUNE against your real floor.

Run it (web stream on so you can watch what the robot sees):
    ./deploy.sh run experimentation/junction_detector.py
    # then open http://<robot-ip>:9000/mjpg and hand-drive over junctions.
    Ctrl + C to quit.
"""

import threading
from time import sleep

import cv2
import numpy as np
from vilib import Vilib

# --- Tuning knobs (calibrate on the real maze) -------------------------------
PROC_W, PROC_H = 160, 120      # downscale frames to this; keeps per-frame work tiny
LOOP_HZ = 8                    # vision rate — junctions don't appear at 30 Hz
NEAR_BAND = (0.72, 0.86)       # fractions of height: "where I am now"
FAR_BAND = (0.30, 0.45)        # fractions of height: look-ahead
MIN_SEG_W = 6                  # px (at PROC_W) — narrower runs are noise
EDGE_MARGIN = 4                # px from a side edge that counts as "touching"


class JunctionState:
    """Latest vision result, shared between the vision worker and the consumer.

    Guarded by a lock so the (future) control loop can read it atomically
    without ever blocking on the camera.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.label = "init"
        self.near_segments = []   # list of (start_x, end_x) in PROC_W pixels
        self.far_segments = []
        self.far_left_touch = False
        self.far_right_touch = False
        self.frames = 0

    def update(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def snapshot(self):
        with self._lock:
            return {
                "label": self.label,
                "near": list(self.near_segments),
                "far": list(self.far_segments),
                "far_left_touch": self.far_left_touch,
                "far_right_touch": self.far_right_touch,
                "frames": self.frames,
            }


def find_segments(profile_bool, min_width):
    """Contiguous runs of True wider than min_width -> list of (start, end) px."""
    segs = []
    start = None
    for x, on in enumerate(profile_bool):
        if on and start is None:
            start = x
        elif not on and start is not None:
            if x - start >= min_width:
                segs.append((start, x - 1))
            start = None
    if start is not None and len(profile_bool) - start >= min_width:
        segs.append((start, len(profile_bool) - 1))
    return segs


def band_profile(mask, band):
    """Collapse a horizontal band to a 1-D boolean column profile.

    A column is True if any row in the band is foreground — robust to a row of
    pixel noise and to the line being slightly off the band's centre.
    """
    y0 = int(band[0] * mask.shape[0])
    y1 = int(band[1] * mask.shape[0])
    return mask[y0:y1, :].max(axis=0) > 0


def classify(far_segs, far_left, far_right):
    """Coarse label from the look-ahead band. Heuristic — refine on real maze."""
    n = len(far_segs)
    if n == 0:
        return "dead_end_or_lost"
    if n >= 3:
        return "cross_or_multi"
    if n == 2:
        return "fork_or_T"
    # exactly one segment ahead
    if far_left and far_right:
        return "horizontal_bar"   # tape spans the view — a T's crossbar
    if far_left:
        return "branch_left"
    if far_right:
        return "branch_right"
    return "straight"


def analyse(frame, state):
    """Threshold one frame, fill `state`. Returns the new label."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (PROC_W, PROC_H))
    # Otsu adapts the white/dark split to current lighting on its own.
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    near = find_segments(band_profile(mask, NEAR_BAND), MIN_SEG_W)
    far = find_segments(band_profile(mask, FAR_BAND), MIN_SEG_W)
    far_left = any(s <= EDGE_MARGIN for s, _ in far)
    far_right = any(e >= PROC_W - 1 - EDGE_MARGIN for _, e in far)
    label = classify(far, far_left, far_right)

    state.update(label=label, near_segments=near, far_segments=far,
                 far_left_touch=far_left, far_right_touch=far_right,
                 frames=state.frames + 1)
    return label


def vision_worker(state, stop_event):
    period = 1.0 / LOOP_HZ
    while not stop_event.is_set():
        frame = getattr(Vilib, "img", None)  # latest frame, set by Vilib's own thread
        if frame is not None and getattr(frame, "size", 0):
            try:
                analyse(frame.copy(), state)
            except Exception as e:  # never let the worker die silently mid-run
                state.update(label=f"error: {e}")
        sleep(period)


def main():
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)  # watch at http://<robot-ip>:9000/mjpg
    sleep(0.8)  # let the camera thread spin up

    state = JunctionState()
    stop_event = threading.Event()
    worker = threading.Thread(target=vision_worker, args=(state, stop_event),
                              daemon=True)
    worker.start()

    print("Junction detector running. Hand-drive over junctions; Ctrl+C to quit.\n")
    last_label = None
    try:
        while True:
            s = state.snapshot()
            # Print on every label change, so the terminal reads like an event log.
            if s["label"] != last_label:
                last_label = s["label"]
                print(f"[{s['frames']:>5}] {s['label']:<16} "
                      f"near={s['near']} far={s['far']} "
                      f"edges(L,R)=({s['far_left_touch']},{s['far_right_touch']})")
            sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        Vilib.camera_close()
        print("\nquit")


if __name__ == "__main__":
    main()
