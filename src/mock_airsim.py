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

class CollisionInfo:
    def __init__(self):
        self.has_collided = False

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
        self.yaw = 0.0  # radians
        self.connected = True
        self.armed = False
        self.flying = False
        
        # Map definition: square room (walls at +/- room_size/2)
        self.room_size = 20.0 
        self.walls = {
            'x_min': -self.room_size / 2,
            'x_max':  self.room_size / 2,
            'y_min': -self.room_size / 2,
            'y_max':  self.room_size / 2
        }
        # Collision state
        self._collision_margin = 0.3  # meters from wall = collision
        self._last_collided = False
        
        print(f"[MOCK] Initialized Virtual Room: {self.room_size}x{self.room_size}m")
        print(f"[MOCK] Walls: X=[{self.walls['x_min']}, {self.walls['x_max']}], "
              f"Y=[{self.walls['y_min']}, {self.walls['y_max']}]")

    def confirmConnection(self):
        print("[MOCK] Connected to Mock AirSim")

    def enableApiControl(self, enable):
        print(f"[MOCK] API Control: {enable}")

    def armDisarm(self, arm):
        self.armed = arm
        print(f"[MOCK] Arm: {arm}")

    def takeoffAsync(self):
        print("[MOCK] Taking off...")
        self.z = -3.0
        self.flying = True
        return AsyncJob()

    def landAsync(self):
        print("[MOCK] Landing...")
        self.z = 0.0
        self.flying = False
        return AsyncJob()

    def hoverAsync(self):
        """Hold current position."""
        return AsyncJob()

    def getMultirotorState(self):
        state = MultirotorState()
        state.kinematics_estimated.position = Vector3r(self.x, self.y, self.z)
        # Set orientation quaternion from yaw (rotation around Z axis)
        # q = (cos(yaw/2), 0, 0, sin(yaw/2))
        state.kinematics_estimated.orientation = Quaternionr(
            w_val=math.cos(self.yaw / 2),
            x_val=0.0,
            y_val=0.0,
            z_val=math.sin(self.yaw / 2)
        )
        return state

    def _check_wall_proximity(self):
        """Check if drone is very close to a wall."""
        margin = self._collision_margin
        if (self.x >= self.walls['x_max'] - margin or
            self.x <= self.walls['x_min'] + margin or
            self.y >= self.walls['y_max'] - margin or
            self.y <= self.walls['y_min'] + margin):
            self._last_collided = True
        else:
            self._last_collided = False

    def moveToPositionAsync(self, x, y, z, velocity):
        # Clamp target to stay within walls (with small margin)
        margin = 0.1
        clamped_x = max(self.walls['x_min'] + margin, min(self.walls['x_max'] - margin, x))
        clamped_y = max(self.walls['y_min'] + margin, min(self.walls['y_max'] - margin, y))
        
        dist = np.sqrt((clamped_x - self.x)**2 + (clamped_y - self.y)**2 + (z - self.z)**2)
        duration = dist / velocity if velocity > 0 else 0
        time.sleep(min(duration, 0.5))  # Cap sleep for faster mock execution
        
        self.x = clamped_x
        self.y = clamped_y
        self.z = z
        
        self._check_wall_proximity()
        
        print(f"[MOCK] Moved to ({self.x:.2f}, {self.y:.2f}, {self.z:.2f})")
        return AsyncJob()

    def moveByVelocityBodyFrameAsync(self, vx, vy, vz, duration):
        """Move in body frame: vx=forward, vy=right, vz=down."""
        dx = (vx * np.cos(self.yaw) - vy * np.sin(self.yaw)) * duration
        dy = (vx * np.sin(self.yaw) + vy * np.cos(self.yaw)) * duration
        
        time.sleep(min(duration, 0.5))
        
        self.x += dx
        self.y += dy
        self.z += vz * duration
        
        # Clamp to walls
        self.x = max(self.walls['x_min'], min(self.walls['x_max'], self.x))
        self.y = max(self.walls['y_min'], min(self.walls['y_max'], self.y))
        
        self._check_wall_proximity()
        return AsyncJob()

    def rotateByYawRateAsync(self, yaw_rate, duration):
        """Rotate by yaw rate (degrees/second) for duration."""
        time.sleep(min(duration, 0.2))
        self.yaw += np.radians(yaw_rate * duration)
        self.yaw = self.yaw % (2 * np.pi)
        print(f"[MOCK] Rotated to Yaw: {np.degrees(self.yaw):.1f} deg")
        return AsyncJob()

    def rotateToYawAsync(self, yaw_deg, duration=1.0):
        """Rotate to absolute yaw angle (degrees)."""
        time.sleep(min(duration, 0.2))
        self.yaw = math.radians(yaw_deg) % (2 * math.pi)
        print(f"[MOCK] Rotated to Yaw: {yaw_deg:.1f} deg")
        return AsyncJob()

    def cancelLastTask(self):
        """Cancel the last async task."""
        pass

    def simGetCollisionInfo(self):
        """Get collision information."""
        info = CollisionInfo()
        info.has_collided = self._last_collided
        return info

    def simPlotLineStrip(self, points, color_rgba=None, thickness=1.0,
                         duration=-1.0, is_persistent=False):
        """Plot a line strip (no-op in mock, just log)."""
        pass

    def simFlushPersistentMarkers(self):
        """Clear all persistent markers (no-op in mock)."""
        pass

    def getLidarData(self, lidar_name=""):
        """
        Simulate 360-degree lidar scan.
        Returns point cloud in body (sensor) frame.
        Rays are cast from drone position in all directions and intersect with room walls.
        """
        data = LidarData()
        points = []
        
        # 360-degree scan with 36 rays (every 10 degrees)
        num_rays = 36
        angles = np.linspace(-np.pi, np.pi, num_rays, endpoint=False)
        lidar_range = 25.0  # meters
        
        for angle in angles:
            # Ray direction in world frame
            ray_yaw = self.yaw + angle
            rx = np.cos(ray_yaw)
            ry = np.sin(ray_yaw)
            
            # Find intersection distances with all 4 walls
            dists = []
            
            # X walls
            if rx != 0:
                t1 = (self.walls['x_max'] - self.x) / rx
                t2 = (self.walls['x_min'] - self.x) / rx
                if t1 > 0.01:
                    dists.append(t1)
                if t2 > 0.01:
                    dists.append(t2)
                    
            # Y walls
            if ry != 0:
                t3 = (self.walls['y_max'] - self.y) / ry
                t4 = (self.walls['y_min'] - self.y) / ry
                if t3 > 0.01:
                    dists.append(t3)
                if t4 > 0.01:
                    dists.append(t4)
            
            if dists:
                min_dist = min(dists)
                if min_dist < lidar_range:
                    # Convert to body (sensor) frame
                    # In body frame: x=forward (along drone heading), y=right
                    lx = min_dist * np.cos(angle)
                    ly = min_dist * np.sin(angle)
                    lz = 0.0
                    points.extend([lx, ly, lz])
        
        data.point_cloud = points
        return data
