# Experimentation

Work-in-progress for maze navigation — junction detection and turn decisions.

This is where camera + grayscale fusion code lives while we develop it:
- **Detection:** spot and classify junctions (T / cross / left / right / dead-end).
- **Policy:** decide which exit to take (start with a fixed-hand rule).

Scripts:
- `junction_detector.py` — vision-only, prints junction classifications. Run
  this first and hand-drive over junctions to calibrate the detection bands.
- `maze_runner.py` — full run: vision + grayscale line-following, always turns
  left at junctions (left-hand rule). Drives motors — claim the robot first.

Stock SunFounder tutorial scripts live in [`../examples/`](../examples/); the
robot API reference is in [`../docs/picar-api.md`](../docs/picar-api.md).
