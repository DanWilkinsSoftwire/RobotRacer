#!/usr/bin/env python3
"""Adaptive grayscale line detection for a varying / low-contrast surface.

A fixed threshold (`value > reference`) fails on this maze: the floor brightness
drifts from area to area, and in some places tape-vs-floor contrast is low. So we
decide "on tape" from CONTRAST measured in real time, not absolute values.

How it works
------------
* Each cycle we sample the background ("floor") as ``min(data)`` — at any moment
  at least one of the three sensors is usually over plain floor, so the darkest
  reading is a good floor estimate. (We use a SHARED floor, not per-sensor,
  because the centre sensor sits on the line most of the time and a per-sensor
  baseline would wrongly learn the line as "background".)
* The floor estimate is smoothed asymmetrically: it drops FAST toward darker
  readings (chase a darkening surface) and rises SLOWLY (so a brief bright
  crossbar / junction can't drag the floor up and blind us).
* ``signal[i] = reading[i] - floor`` is how far each sensor stands above floor.
* A sensor is "on tape" when its signal clears BOTH a small absolute noise gate
  (``min_contrast``) AND a fraction of the strongest sensor's signal (``frac``).
  The relative part is what makes it work in low-contrast patches.

This handles: surface drift (floor follows it), brief junctions (slow rise keeps
the floor put), and low contrast (relative threshold with only a small absolute
floor). It deliberately knows nothing about junctions — callers interpret the
[left, mid, right] booleans (e.g. "both outer on tape" == junction).
"""


class AdaptiveLine:
    def __init__(self, floor_drop=0.3, floor_rise=0.01, min_contrast=30, frac=0.5):
        """All knobs are in raw ADC counts / fractions; tune on the real maze.

        floor_drop   : EMA rate when the new floor sample is DARKER (fast, 0..1).
        floor_rise   : EMA rate when it is BRIGHTER (slow — resist bright tape).
        min_contrast : ADC counts a sensor must exceed the floor by to count at
                       all. Set just above sensor noise; lower = more sensitive in
                       low-contrast areas, but noisier.
        frac         : a sensor is "on" if its signal >= frac * strongest signal.
        """
        self.floor = None
        self.floor_drop = floor_drop
        self.floor_rise = floor_rise
        self.min_contrast = min_contrast
        self.frac = frac

    def update(self, data):
        """Feed one [left, mid, right] reading.

        Returns (on_tape, signal, floor):
            on_tape : [bool, bool, bool]  — which sensors see tape
            signal  : [float, float, float] — each sensor's level above floor
            floor   : float — current adaptive background estimate
        """
        sample = min(data)
        if self.floor is None:
            self.floor = float(sample)
        rate = self.floor_drop if sample < self.floor else self.floor_rise
        self.floor += rate * (sample - self.floor)

        signal = [max(0.0, v - self.floor) for v in data]
        peak = max(signal)
        if peak < self.min_contrast:
            on_tape = [False, False, False]          # nothing stands out -> lost / all floor
        else:
            thresh = max(self.min_contrast, self.frac * peak)
            on_tape = [s >= thresh for s in signal]
        return on_tape, signal, self.floor
