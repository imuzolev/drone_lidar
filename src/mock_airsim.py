import time
import numpy as np
import math

class Vector3r:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x_val = x
        self.y_val = y
        self.z_val = z

class Quaternionr:
    def __init__(self, w_val=1.0, x_val=0.0, y_val=0.0, z_val=0.0):
        self.w_val = w_val
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

class KinematicsState:
    def __init__(self):
        self.position = Vector3r()
        self.orientation = Quaternionr()

class MultirotorState:
    def __init__(self):
        self.kinematics_estimated = KinematicsState()

class LidarData:
    def __init__(self):
        self.point_cloud = []

class AsyncJob:
    def join(self):
        pass

class MultirotorClient:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0 # radians
        self.connected = True
        
        # Map definition: 20x20m room (walls at +/- 10m)
        self.room_size = 20.0 
        self.walls = {
            'x_min': -self.room_size/2,
            'x_max': self.room_size/2,
            'y_min': -self.room_size/2,
            'y_max': self.room_size/2
        }
        print(f"[MOCK] Initialized Virtual Room: {self.room_size}x{self.room_size}m")

    def confirmConnection(self):
        print("[MOCK] Connected to Mock AirSim")

    def enableApiControl(self, enable):
        print(f"[MOCK] API Control: {enable}")

    def armDisarm(self, arm):
        print(f"[MOCK] Arm: {arm}")

    def takeoffAsync(self):
        print("[MOCK] Taking off...")
        self.z = -3.0
        return AsyncJob()

    def landAsync(self):
        print("[MOCK] Landing...")
        self.z = 0.0
        return AsyncJob()

    def getMultirotorState(self):
        state = MultirotorState()
        state.kinematics_estimated.position = Vector3r(self.x, self.y, self.z)
        return state

    def moveToPositionAsync(self, x, y, z, velocity):
        print(f"[MOCK] Moving to ({x:.2f}, {y:.2f}, {z:.2f}) at {velocity} m/s")
        # Calculate duration
        dist = np.sqrt((x-self.x)**2 + (y-self.y)**2 + (z-self.z)**2)
        duration = dist / velocity if velocity > 0 else 0
        time.sleep(duration)
        self.x = x
        self.y = y
        self.z = z
        return AsyncJob()

    def moveByVelocityBodyFrameAsync(self, vx, vy, vz, duration):
        # Update position based on body frame velocity
        # vx is forward, vy is right
        
        # Convert to world frame
        # x_world = vx * cos(yaw) - vy * sin(yaw)
        # y_world = vx * sin(yaw) + vy * cos(yaw)
        
        dx = (vx * np.cos(self.yaw) - vy * np.sin(self.yaw)) * duration
        dy = (vx * np.sin(self.yaw) + vy * np.cos(self.yaw)) * duration
        
        time.sleep(duration)
        
        self.x += dx
        self.y += dy
        self.z += vz * duration
        
        # Clamp to walls (collision)
        self.x = max(self.walls['x_min'], min(self.walls['x_max'], self.x))
        self.y = max(self.walls['y_min'], min(self.walls['y_max'], self.y))
        
        # print(f"[MOCK] Moved to ({self.x:.2f}, {self.y:.2f}) Yaw: {math.degrees(self.yaw):.1f}")
        return AsyncJob()

    def rotateByYawRateAsync(self, yaw_rate, duration):
        # yaw_rate in degrees/s
        time.sleep(duration)
        self.yaw += np.radians(yaw_rate * duration)
        self.yaw = self.yaw % (2 * np.pi)
        print(f"[MOCK] Rotated to Yaw: {np.degrees(self.yaw):.1f} deg")
        return AsyncJob()

    def getLidarData(self, lidar_name=""):
        data = LidarData()
        points = []
        
        # Simulate rays
        # We need to find intersection with walls
        # Simple ray casting in front sector
        
        # Angles to check (relative to drone yaw): -45 to 45 deg
        angles = np.linspace(-np.pi/4, np.pi/4, 10)
        
        for angle in angles:
            ray_yaw = self.yaw + angle
            
            # Ray direction
            rx = np.cos(ray_yaw)
            ry = np.sin(ray_yaw)
            
            # Find distance to walls
            # x = x0 + t * rx
            # y = y0 + t * ry
            
            dists = []
            
            # X walls
            if rx != 0:
                t1 = (self.walls['x_max'] - self.x) / rx
                t2 = (self.walls['x_min'] - self.x) / rx
                if t1 > 0: dists.append(t1)
                if t2 > 0: dists.append(t2)
                
            # Y walls
            if ry != 0:
                t3 = (self.walls['y_max'] - self.y) / ry
                t4 = (self.walls['y_min'] - self.y) / ry
                if t3 > 0: dists.append(t3)
                if t4 > 0: dists.append(t4)
            
            if dists:
                min_dist = min(dists)
                if min_dist < 20: # Lidar range
                    # Convert back to sensor frame (local point cloud)
                    # Point in world: px = x + min_dist * rx, py = y + min_dist * ry
                    # We need it in body frame (where x is forward)
                    # local_x = dist * cos(angle)
                    # local_y = dist * sin(angle)
                    
                    lx = min_dist * np.cos(angle)
                    ly = min_dist * np.sin(angle)
                    lz = 0
                    
                    points.extend([lx, ly, lz])
        
        data.point_cloud = points
        return data
