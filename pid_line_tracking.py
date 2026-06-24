#!/usr/bin/env python3
"""Robust PID Line Tracking with Sensor Normalization.

This script implements continuous line tracking using:
1. Grayscale reading normalization based on minimum and maximum calibration limits.
2. Weighted average error calculation for continuous position estimation.
3. A PID controller for smooth, oscillation-free steering.
4. Dynamic speed regulation to slow down on curves.
"""

import os
import sys
import time
import math

# Repo root on the path so `from config import CONFIG` works
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from picarx import Picarx
from config import CONFIG

def main():
    px = Picarx()
    
    # Load configuration parameters
    grayscale_min = CONFIG.get("grayscale_min", [300, 300, 300])
    grayscale_max = CONFIG.get("grayscale_max", [900, 900, 900])
    kp = CONFIG.get("pid_kp", 40.0)
    ki = CONFIG.get("pid_ki", 0.0)
    kd = CONFIG.get("pid_kd", 5.0)
    base_speed = CONFIG.get("base_speed", 10)
    speed_scale = CONFIG.get("speed_scale", 0.5)
    
    print("PID Line Tracking Initialized:")
    print(f"  Min/Max: {grayscale_min} / {grayscale_max}")
    print(f"  PID gains: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"  Speed: base={base_speed}, scale={speed_scale}\n")

    # PID state variables
    integral = 0.0
    last_error = 0.0
    last_time = time.monotonic()
    
    # Tracking control variables
    lost_start_time = None
    loop_hz = 50
    period = 1.0 / loop_hz
    
    try:
        while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt <= 0:
                dt = 0.001

            # Read raw grayscale values
            raw = px.get_grayscale_data()
            
            # Normalize readings to [0.0, 1.0] range
            normalized = []
            for i in range(3):
                val = raw[i]
                min_v = grayscale_min[i]
                max_v = grayscale_max[i]
                
                if max_v <= min_v:
                    norm_val = 0.0
                else:
                    norm_val = (val - min_v) / (max_v - min_v)
                    norm_val = max(0.0, min(1.0, norm_val))
                normalized.append(norm_val)
                
            L, M, R = normalized
            total_intensity = L + M + R
            
            # Line Loss Check
            # If total normalized intensity is very low, we are not on the white tape
            if total_intensity < 0.2:
                if lost_start_time is None:
                    lost_start_time = now
                elif now - lost_start_time > 1.0: # Stop after being lost for 1 second
                    print(f"Line lost! (Intensity: {total_intensity:.3f}) Stopping.")
                    break
                
                # Keep moving slowly with last known steering angle to try to find the line
                px.forward(max(1, base_speed // 2))
                sleep(period)
                continue
            else:
                lost_start_time = None

            # Compute line error (position offset): range is [-1.0, 1.0]
            # Left dominated -> error < 0; Right dominated -> error > 0
            error = (R - L) / total_intensity

            # --- PID Calculations ---
            # Proportional term
            p_term = kp * error
            
            # Integral term (with windup clamp)
            integral += error * dt
            integral = max(-5.0, min(5.0, integral))
            i_term = ki * integral
            
            # Derivative term
            derivative = (error - last_error) / dt
            d_term = kd * derivative
            
            # Calculate output steering angle
            steer_angle = p_term + i_term + d_term
            steer_angle = max(-30.0, min(30.0, steer_angle))
            
            last_error = error
            
            # --- Dynamic Speed Regulation ---
            # Speed drops as absolute error grows
            speed = base_speed * (1.0 - speed_scale * abs(error))
            speed = max(1, min(100, round(speed)))
            
            # Drive the hardware
            px.set_dir_servo_angle(steer_angle)
            px.forward(speed)
            
            # Verbose status reporting
            print(f"Raw: {raw} | Norm: [{L:.2f}, {M:.2f}, {R:.2f}] | Error: {error:+.3f} | Steer: {steer_angle:+.1f}° | Speed: {speed}", end="\r", flush=True)
            
            sleep(period)
            
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stopping and exiting.")
    finally:
        px.stop()
        print("\nMotors stopped.")

if __name__ == "__main__":
    main()
