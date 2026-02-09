import time
import numpy as np
import math
import random
import sys
import argparse

# Try to import airsim, but allow fallback
try:
    import airsim
except ImportError:
    airsim = None

import mock_airsim

class AutonomousExplorer:
    def __init__(self, use_mock=False):
        self.use_mock = use_mock
        self.client = None
        
        if not self.use_mock and airsim is not None:
            try:
                print("[INFO] Connecting to AirSim...")
                self.client = airsim.MultirotorClient()
                self.client.confirmConnection()
                print("[INFO] Connected to real AirSim.")
            except Exception as e:
                print(f"[WARN] Failed to connect to AirSim: {e}")
                print("[INFO] Falling back to Mock AirSim.")
                self.use_mock = True
        else:
            self.use_mock = True
            
        if self.use_mock:
            self.client = mock_airsim.MultirotorClient()
            self.client.confirmConnection()

        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        # Flight parameters
        self.target_altitude = -3.0  # 3 meters up (NED z is negative)
        self.safe_distance = 2.0     # 2 meters buffer
        self.speed = 2.0             # 2 m/s
        self.rotation_speed = 30     # degrees per second
        self.exploration_time = 15   # seconds to explore (reduced for demo)
        
        self.start_position = None
        self.is_running = False

    def takeoff(self):
        print("[INFO] Taking off...")
        self.client.takeoffAsync().join()
        
        # Record start position
        state = self.client.getMultirotorState()
        self.start_position = state.kinematics_estimated.position
        print(f"[INFO] Start Position: ({self.start_position.x_val:.2f}, {self.start_position.y_val:.2f})")
        
        # Ascend to target altitude
        print(f"[INFO] Ascending to {abs(self.target_altitude)}m...")
        self.client.moveToPositionAsync(
            self.start_position.x_val, 
            self.start_position.y_val, 
            self.target_altitude, 
            2.0
        ).join()
        print("[INFO] Reached target altitude.")

    def get_lidar_distance(self):
        """
        Returns the minimum distance to an obstacle in the front sector.
        """
        try:
            lidar_data = self.client.getLidarData()
        except Exception as e:
            if "No lidar with name" in str(e):
                print("[ERROR] Lidar sensor not found on drone!")
                print("[HELP] Please add Lidar configuration to your settings.json.")
                print("[HELP] See docs/AIRSIM_SETUP.md for instructions.")
                # Fallback to infinity (no obstacle) or raise specific error to switch to mock
                # For safety, we should probably stop or switch to mock.
                # Here we raise an exception to be caught by run()
                raise RuntimeError("Lidar not configured")
            raise e
        
        if len(lidar_data.point_cloud) < 3:
            return float('inf')

        # Parse point cloud (x, y, z floats)
        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
        
        # Filter points:
        # 1. We care about obstacles at our flight level (e.g. +/- 1m from drone Z)
        #    Note: Lidar points are in sensor frame. If sensor is horizontal, Z~0 is the plane.
        # 2. We care about points in FRONT of the drone (X > 0)
        # 3. We care about a cone (e.g. Y between -X and X for +/- 45 degrees)
        
        # Filter by Z (height relative to sensor) - assume walls are tall enough
        # Let's just look at XY distance for now.
        
        # Filter for front sector: X > 0 and abs(Y) < X (approx +/- 45 deg)
        front_points = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < points[:, 0])]
        
        if len(front_points) == 0:
            return float('inf')
            
        # Calculate Euclidean distance in XY plane
        distances = np.linalg.norm(front_points[:, :2], axis=1)
        
        if len(distances) == 0:
            return float('inf')
            
        return np.min(distances)

    def rotate_to_clear(self):
        """
        Rotates the drone until the path is clear.
        """
        print("[INFO] Obstacle detected! Searching for clear path...")
        
        # Stop first
        self.client.moveByVelocityBodyFrameAsync(0, 0, 0, 1).join()
        
        # Rotate randomly left or right
        direction = random.choice([-1, 1])
        duration = 1.0 # Rotate for 1 second chunks
        
        for i in range(10): # Try 10 times
            # Rotate
            yaw_rate = direction * self.rotation_speed
            self.client.rotateByYawRateAsync(yaw_rate, duration).join()
            
            # Check Lidar
            dist = self.get_lidar_distance()
            if dist > self.safe_distance + 1.0: # Add hysteresis
                print(f"[INFO] Path clear ({dist:.2f}m). Resuming.")
                return
                
        print("[WARN] Could not find clear path after rotation.")

    def return_home(self):
        print("[INFO] Returning to start position...")
        if self.start_position:
            # Go to start X, Y at current altitude
            self.client.moveToPositionAsync(
                self.start_position.x_val, 
                self.start_position.y_val, 
                self.target_altitude, 
                self.speed
            ).join()
            
            # Land
            print("[INFO] Landing...")
            self.client.landAsync().join()
            print("[INFO] Landed.")
        else:
            print("[ERROR] Start position not recorded.")

    def run(self):
        try:
            self.takeoff()
            
            start_time = time.time()
            print(f"[INFO] Starting exploration for {self.exploration_time} seconds...")
            
            while time.time() - start_time < self.exploration_time:
                # 1. Check Lidar
                dist = self.get_lidar_distance()
                
                # Log current state
                state = self.client.getMultirotorState()
                pos = state.kinematics_estimated.position
                # print(f"[DEBUG] Pos: ({pos.x_val:.1f}, {pos.y_val:.1f}) Dist: {dist:.1f}m")

                if dist < self.safe_distance:
                    print(f"[WARN] Wall detected at {dist:.2f}m! Pos: ({pos.x_val:.1f}, {pos.y_val:.1f})")
                    self.rotate_to_clear()
                else:
                    # 2. Move Forward
                    # We use moveByVelocityBodyFrameAsync to move relative to drone heading
                    # vx = speed, vy = 0, vz = 0, duration = small step
                    self.client.moveByVelocityBodyFrameAsync(self.speed, 0, 0, 0.5).join()
                    
            print("[INFO] Exploration time finished.")
            self.return_home()
            
        except Exception as e:
            print(f"[ERROR] An error occurred: {e}")
            import traceback
            traceback.print_exc()
            # Try to land safely
            self.client.landAsync().join()
        finally:
            self.client.armDisarm(False)
            self.client.enableApiControl(False)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mock", action="store_true", help="Use mock AirSim environment")
    args = parser.parse_args()
    
    explorer = AutonomousExplorer(use_mock=args.mock)
    explorer.run()
