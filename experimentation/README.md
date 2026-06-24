# Experimentation

Work-in-progress for maze navigation — junction detection and turn decisions.

This is where camera + grayscale fusion code lives while we develop it:
- **Detection:** spot and classify junctions (T / cross / left / right / dead-end).
- **Policy:** decide which exit to take (start with a fixed-hand rule).

Stock SunFounder tutorial scripts live in [`../examples/`](../examples/); the
robot API reference is in [`../docs/picar-api.md`](../docs/picar-api.md).
