import sys
import os
import math

# Mocking Picarx for local unit test
class MockPicarx:
    def __init__(self):
        self.steer = 0.0
        self.speed = 0
        self.is_stopped = False
        
    def get_grayscale_data(self):
        # Center line simulation (should yield error near 0)
        return [600, 500, 600] 

    def set_dir_servo_angle(self, val):
        self.steer = val

    def forward(self, val):
        self.speed = val

    def stop(self):
        self.is_stopped = True

def run_pid_iteration(raw, grayscale_min, grayscale_max, kp, ki, kd, base_speed, speed_scale, last_error, integral, dt):
    # Normalized reading calculation
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
    
    if total_intensity < 0.2:
        return None, last_error, integral, 0
        
    error = (R - L) / total_intensity

    # PID calculation
    p_term = kp * error
    integral += error * dt
    integral = max(-5.0, min(5.0, integral))
    i_term = ki * integral
    
    derivative = (error - last_error) / dt
    d_term = kd * derivative
    
    steer_angle = p_term + i_term + d_term
    steer_angle = max(-30.0, min(30.0, steer_angle))
    
    # Speed regulation
    speed = base_speed * (1.0 - speed_scale * abs(error))
    speed = max(1, min(100, round(speed)))
    
    return steer_angle, error, integral, speed

def main():
    # Test case 1: Center line test
    # Min/max references
    min_v = [302, 285, 422]
    max_v = [913, 775, 968]
    
    # Normalized: L=0.5, M=0.5, R=0.5 -> error=0.0 -> steer=0.0 -> speed=10
    raw_center = [
        int(min_v[0] + 0.5 * (max_v[0] - min_v[0])),
        int(min_v[1] + 0.5 * (max_v[1] - min_v[1])),
        int(min_v[2] + 0.5 * (max_v[2] - min_v[2]))
    ]
    
    steer, err, integ, speed = run_pid_iteration(
        raw_center, min_v, max_v, kp=40.0, ki=0.0, kd=5.0, 
        base_speed=10, speed_scale=0.5, last_error=0.0, integral=0.0, dt=0.02
    )
    
    assert abs(err) < 0.05, f"Center error expected near 0, got {err}"
    assert abs(steer) < 0.2, f"Center steer expected near 0, got {steer}"
    assert speed == 10, f"Center speed expected 10, got {speed}"
    
    # Test case 2: Right drift (line to the right -> R value is higher than L)
    # L=0.2, M=0.5, R=0.8
    raw_right = [
        int(min_v[0] + 0.2 * (max_v[0] - min_v[0])),
        int(min_v[1] + 0.5 * (max_v[1] - min_v[1])),
        int(min_v[2] + 0.8 * (max_v[2] - min_v[2]))
    ]
    
    steer, err, integ, speed = run_pid_iteration(
        raw_right, min_v, max_v, kp=40.0, ki=0.0, kd=5.0, 
        base_speed=10, speed_scale=0.5, last_error=0.0, integral=0.0, dt=0.02
    )
    
    assert err > 0.0, f"Right drift error should be positive, got {err}"
    assert steer > 0.0, f"Right drift steer correction should be positive (steer left), got {steer}"
    assert speed < 10, f"Right drift speed should be less than 10, got {speed}"
    
    print("PID iteration logic tests passed successfully!")

if __name__ == "__main__":
    main()
