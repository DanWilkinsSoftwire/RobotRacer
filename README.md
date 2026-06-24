# PiCar Maze

Team project: navigate a maze (white tape on a dark floor) with a SunFounder
PiCar-X. See [`docs/picar-api.md`](docs/picar-api.md) for the robot API.

## Workflow at a glance

- **Source of truth:** this GitHub repo. Branch → PR → merge to `main`.
- **Edit:** on your own laptop.
- **Deploy:** `./deploy.sh` pushes your current branch and makes the robot check
  it out via git — so the device only ever runs tracked, committed code.
- **One shared robot:** coordinate driving time.

## First-time setup

### Each teammate (laptop)
1. Clone the repo.
2. Make sure you can reach the robot over SSH:
   ```bash
   ssh pi@picar.local        # or pi@<robot-ip>
   ```
   If `picar.local` doesn't resolve, find the IP and use:
   ```bash
   PICAR_HOST=pi@192.168.1.42 ./deploy.sh
   ```
   (Add an entry to `~/.ssh/config` and set up SSH keys so you're not typing
   passwords on every deploy.)

### The robot (once)
The device must hold a git clone of this repo at `~/picar` so `deploy.sh` can
fetch into it:
```bash
ssh pi@picar.local 'git clone https://github.com/DanWilkinsSoftwire/RobotRacer ~/picar'
```

## Daily loop

A commit is the unit of deploy — uncommitted edits won't ship (deploy.sh stops
if your tree is dirty), so the robot never runs anything that isn't in git.

```bash
git pull                            # get teammates' changes first
# ... edit code locally ...
git commit -am "tune steer offset"  # commit your change (even WIP)
./deploy.sh run examples/6.line_tracking.py  # push branch, robot checks it out, then runs it
```

`./deploy.sh` (no args) pushes + syncs the robot without running anything.

## The two team rules that prevent pain

### 1. Calibration lives in git
`config.json` holds our tuning (`line_reference`, `drive_power`, `steer_offset`)
for the one robot on the one maze floor. It's **committed** — when you dial in a
better value, commit and push so everyone gets it on the next `git pull`.

- Read it in code via `from config import CONFIG`.
- `deploy.sh` syncs `config.json` to the robot like any other file.
- Note: the robot's servo/motor calibration (`/opt/picar-x/picar-x.conf`, written
  by the SunFounder calibration scripts) lives on the device, not in this repo,
  so it isn't affected either way.

### 2. Coordinate the robot
There's one robot. Before you deploy-and-drive, claim it in the team chat
("robot 2–3pm"). Two people deploying at once will fight over the motors.

## Branching

Work on a branch, open a PR, keep `main` deployable:
```bash
git checkout -b steering-tuning
# ...work...
git push -u origin steering-tuning   # then open a PR
```
