#!/usr/bin/env python3
"""Adaptive grayscale line detection for a varying / low-contrast surface.

A fixed threshold (`value > reference`) fails on this maze: the floor brightness
drifts area to area. But the TAPE reads consistently (~1000 in testing), so we
anchor to that and only track the moving part — the floor:

    threshold = floor + max(min_contrast, frac * (tape_level - floor))

With frac=0.5 that's the midpoint between the current floor and the tape level.
A sensor on tape (~1000) sits far above it; the floor and its background wobble
sit far below it; so small background variation can't be mistaken for tape.

Tracking the floor:
    Each cycle the floor sample is the darkest sensor whose reading is BELOW the
    current threshold (i.e. not "on tape"). The centre sensor sitting on the line
    is above the threshold and so is ignored — the off-line sensors supply the
    background — and the floor estimate stays correct even on long straights and
    even as the floor brightens (it can always track up to just under the
    threshold, so it never stalls). It drops FAST toward darker readings and
    rises a bit slower toward brighter ones. When all sensors are on tape (a
    crossbar) there is no sub-threshold sample, so the floor simply holds.

The detector knows nothing about junctions — callers interpret the
[left, mid, right] booleans (e.g. "both outer on tape" == junction).
"""


class AdaptiveLine:
    def __init__(self, tape_level=1000, frac=0.5, min_contrast=40,
                 floor_drop=0.3, floor_rise=0.1):
        """Knobs are in raw ADC counts / fractions; tune on the real maze.

        tape_level   : the (consistent) tape reading, ~1000 here.
        frac         : threshold sits frac of the way from floor up to tape_level.
        min_contrast : absolute minimum gap above the floor (ADC counts), so very
                       low contrast still needs a real margin to trip.
        floor_drop   : EMA rate chasing a DARKER floor (fast, 0..1).
        floor_rise   : EMA rate tracking a BRIGHTER floor.
        """
        self.tape_level = float(tape_level)
        self.frac = frac
        self.min_contrast = min_contrast
        self.floor_drop = floor_drop
        self.floor_rise = floor_rise
        self.floor = None

    def seed_floor(self, samples):
        """Seed the floor from startup readings: the darkest value seen."""
        vals = [v for row in samples for v in row]
        if vals:
            self.floor = float(min(vals))

    def threshold(self):
        f = self.floor if self.floor is not None else 0.0
        return f + max(self.min_contrast, self.frac * (self.tape_level - f))

    def update(self, data):
        """Feed one [left, mid, right] reading.

        Returns (on_tape, threshold, floor):
            on_tape   : [bool, bool, bool] — which sensors see tape
            threshold : float — current on-threshold
            floor     : float — current background estimate
        """
        if self.floor is None:                       # seed from first frame
            self.floor = float(min(data))

        thr = self.threshold()
        on = [v >= thr for v in data]

        # Track the floor from the darkest sub-threshold (off-line) sensor.
        bg = [v for v in data if v < thr]
        if bg:
            sample = min(bg)
            rate = self.floor_drop if sample < self.floor else self.floor_rise
            self.floor += rate * (sample - self.floor)

        return on, thr, self.floor
