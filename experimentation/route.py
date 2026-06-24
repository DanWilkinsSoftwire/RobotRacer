"""Predefined route for THIS maze: one action per junction, executed in order.

Actions:
    "straight" — drive on through the junction
    "left"     — pivot left  onto the left branch
    "right"    — pivot right onto the right branch
    "uturn"    — spin ~180 (dead end)
    "stop"     — arrived; halt

We just COUNT junctions (no type checking), so the Nth junction the robot
reaches executes ROUTE[N]. Keep this in sync with the physical maze — a miscount
desyncs everything after it. The easiest way to derive it: do a mapping run with
maze_runner_mapped.py (it logs the junction sequence) or walk the maze by hand.

This is maze-specific data, like config.json — commit it.
"""

ROUTE = [
    "straight",
    "left",
    "right",
    "straight",
    "stop",
]
