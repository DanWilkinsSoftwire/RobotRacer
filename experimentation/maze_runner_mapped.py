#!/usr/bin/env python3
"""Maze runner with topological and coordinate mapping.

This script extends the basic left-hand rule maze runner by tracking the robot's
location (via dead reckoning) and recording junctions (nodes) and paths (edges).
It supports saving the map to a JSON file and loading it to resume mapping in sections.

Run it:
    ./deploy.sh run experimentation/maze_runner_mapped.py --map my_maze.json
"""

import os
import sys
import threading
import math
import argparse
from time import monotonic, sleep

# Repo root on the path so `from config import CONFIG` works
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from picarx import Picarx
from vilib import Vilib

import config
from junction_detector import JunctionState, vision_worker
from maze_map import MazeMap

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config.json")

# Live config helper
_cfg = config.load()
_cfg_mtime = os.stat(CONFIG_PATH).st_mtime_ns

def live_config():
    global _cfg, _cfg_mtime
    try:
        m = os.stat(CONFIG_PATH).st_mtime_ns
        if m != _cfg_mtime:
            _cfg_mtime = m
            _cfg = config.load()
    except Exception:
        pass
    return _cfg

# --- Tuning knobs ------------------------------------------------------------
FOLLOW_HZ = 50                 # control loop rate
TURN_ANGLE = 30                # sharp steer for a committed junction turn ([-30,30], + = left)
MIN_TURN_S = 0.4               # don't declare the turn "done" before this
MAX_TURN_S = 2.5               # give up turning after this (avoid spinning forever)
COOLDOWN_S = 1.0               # after a turn, ignore junctions briefly so we don't re-fire

# Vision labels that mean "a junction with a possible LEFT path is ahead".
JUNCTION_LABELS = {"fork_or_T", "cross_or_multi", "branch_left", "horizontal_bar"}

# --- Dead Reckoning Calibration Constants ------------------------------------
# Estimate linear speed (meters per second) at DRIVE_POWER=10
SPEED_M_S = 0.08  
# Estimate yaw rate (radians per second per degree of steering) while following
STEER_YAW_RATE_FACTOR = 0.003
# Estimate yaw rate (radians per second) when steering sharp left during turn
TURN_YAW_RATE = 1.1  

def read_on_tape(px, reference):
    """[left, mid, right] booleans: True where a sensor sees white tape."""
    data = px.get_grayscale_data()
    return [v > r for v, r in zip(data, reference)], data

def follow_steer(on_tape, last_steer, offset):
    """Steering angle for normal line-following from the on-tape booleans."""
    left, mid, right = on_tape
    if mid and not left and not right:
        return 0                       # centred
    if left and not right:
        return offset            # tape to the left -> steer left
    if right and not left:
        return -offset           # tape to the right -> steer right
    if not any(on_tape):
        return last_steer              # lost the line -> keep last correction (recovery)
    return 0                           # ambiguous -> hold straight

def main():
    parser = argparse.ArgumentParser(description="Mapped PiCar-X Maze Runner")
    parser.add_argument("--map", type=str, default="maze_map.json", help="Path to JSON map file")
    parser.add_argument("--start-node", type=str, default=None, help="Existing node ID to start/resume from")
    args = parser.parse_args()

    px = Picarx()
    cfg = live_config()
    reference = cfg["line_reference"]
    px.set_line_reference(reference)

    # Initialize or load map
    maze_map = MazeMap()
    map_loaded = False
    if os.path.exists(args.map):
        print(f"Loading existing map from {args.map}...")
        map_loaded = maze_map.load_from_file(args.map)
        
    # State values
    x, y = 0.0, 0.0
    theta = 0.0  # 0 = North, negative = counter-clockwise (West), positive = clockwise (East)
    current_node = None
    last_action = "start"

    if map_loaded:
        # Determine start node
        if args.start_node:
            if args.start_node in maze_map.nodes:
                current_node = args.start_node
                node_data = maze_map.get_node(current_node)
                x = node_data["x"]
                y = node_data["y"]
                print(f"Resuming mapping from node '{current_node}' at ({x}, {y})")
            else:
                print(f"Warning: Start node '{args.start_node}' not found in map. Starting from scratch.")
        
        if current_node is None:
            # If not specified or not found, default to the last node
            node_ids = sorted(list(maze_map.nodes.keys()))
            if node_ids:
                current_node = node_ids[-1]
                node_data = maze_map.get_node(current_node)
                x = node_data["x"]
                y = node_data["y"]
                print(f"No start-node specified. Defaulting to last node '{current_node}' at ({x}, {y})")

    if current_node is None:
        # Start new map or fallback
        current_node = maze_map.add_node("start", 0.0, 0.0, node_id="node_0")
        print(f"Initialized new map with starting node '{current_node}' at (0, 0)")
        maze_map.save_to_file(args.map)

    # Start camera and vision worker thread
    Vilib.camera_start(vflip=False, hflip=False)
    Vilib.display(local=False, web=True)
    sleep(0.8)

    state = JunctionState()
    stop_event = threading.Event()
    worker = threading.Thread(target=vision_worker, args=(state, stop_event), daemon=True)
    worker.start()

    period = 1.0 / FOLLOW_HZ
    mode = "follow"           # "follow" | "turn_left"
    last_steer = 0
    turn_start = 0.0
    cooldown_until = 0.0

    segment_start_time = monotonic()
    segment_start_x = x
    segment_start_y = y
    last_time = monotonic()

    print(f"Mapped Maze Runner started. Saving to '{args.map}'. Ctrl+C to stop.\n")
    try:
        cfg = live_config()
        px.forward(cfg["drive_power"])
        while True:
            now = monotonic()
            dt = now - last_time
            last_time = now

            cfg = live_config()
            power = cfg["drive_power"]
            offset = cfg["steer_offset"]
            reference = cfg["line_reference"]

            on_tape, raw = read_on_tape(px, reference)
            js = state.snapshot()

            # --- Dead Reckoning Integration ---
            # Power is power, so speed is SPEED_M_S
            # Adjust speed based on whether we are moving
            current_speed = SPEED_M_S if mode != "stop" else 0.0
            
            if mode == "follow":
                steer = follow_steer(on_tape, last_steer, offset)
                last_steer = steer
                
                # Estimate yaw change based on steering correction
                theta -= (steer * STEER_YAW_RATE_FACTOR) * dt
                
                x += current_speed * math.sin(theta) * dt
                y += current_speed * math.cos(theta) * dt
                
                px.set_dir_servo_angle(steer)
                px.forward(power)

                # Check for junction detection
                junction_left = (js["label"] in JUNCTION_LABELS and on_tape[0])
                if now >= cooldown_until and junction_left:
                    # We reached a junction!
                    px.stop()
                    sleep(0.1) # Brief pause to stabilize
                    
                    duration = now - segment_start_time
                    distance = math.sqrt((x - segment_start_x)**2 + (y - segment_start_y)**2)
                    
                    # Log the new junction node
                    new_node = maze_map.add_node(js["label"], x, y)
                    maze_map.add_edge(current_node, new_node, last_action, duration, distance)
                    print(f"-> Junction '{new_node}' ({js['label']}) at ({x:.2f}, {y:.2f}) raw={raw}: duration={duration:.2f}s")
                    
                    # Autosave map
                    maze_map.save_to_file(args.map)
                    
                    # Update transition states
                    current_node = new_node
                    last_action = "turn_left" # The action we are about to execute
                    
                    # Start turn left
                    mode = "turn_left"
                    turn_start = now
                    px.set_dir_servo_angle(TURN_ANGLE)
                    px.forward(power)

            elif mode == "turn_left":
                # Turning left decreases theta (heading turns to West/counter-clockwise)
                theta -= TURN_YAW_RATE * dt
                
                x += current_speed * math.sin(theta) * dt
                y += current_speed * math.cos(theta) * dt
                
                elapsed = now - turn_start
                centred = on_tape == [False, True, False]
                
                if elapsed >= MIN_TURN_S and centred:
                    # Turn complete: snap theta to the nearest cardinal 90 degree direction
                    theta = round(theta / (math.pi / 2)) * (math.pi / 2)
                    # Normalize theta between -pi and pi
                    theta = (theta + math.pi) % (2 * math.pi) - math.pi
                    
                    mode = "follow"
                    cooldown_until = now + COOLDOWN_S
                    last_steer = 0
                    px.set_dir_servo_angle(0)
                    
                    segment_start_time = now
                    segment_start_x = x
                    segment_start_y = y
                    
                    print(f"   Turn complete in {elapsed:.1f}s. Heading snapped to {math.degrees(theta):.1f}°")
                    
                elif elapsed >= MAX_TURN_S:
                    # Turn timeout: fallback to follow mode
                    theta = round(theta / (math.pi / 2)) * (math.pi / 2)
                    theta = (theta + math.pi) % (2 * math.pi) - math.pi
                    
                    mode = "follow"
                    cooldown_until = now + COOLDOWN_S
                    last_steer = 0
                    px.set_dir_servo_angle(0)
                    
                    segment_start_time = now
                    segment_start_x = x
                    segment_start_y = y
                    
                    print(f"   Turn TIMED OUT after {elapsed:.1f}s — resuming follow. Heading snapped to {math.degrees(theta):.1f}°")

            sleep(period)
            
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        px.stop()
        Vilib.camera_close()
        # Save one final snapshot of the map
        maze_map.save_to_file(args.map)
        print(f"\nStopped. Map saved to '{args.map}'")

if __name__ == "__main__":
    main()
