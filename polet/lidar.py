"""
Lidar point cloud processing.
"""

import math
import numpy as np


def get_lidar_points_world(self):
    """Get lidar point cloud converted to world coordinates."""
    for attempt in range(2):
        try:
            lidar_data = self.client.getLidarData()
            break
        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("getLidarData")
                continue
            self._log_throttled_error("get_lidar_data", "getLidarData failed", e, interval_s=3.0)
            return np.array([]).reshape(0, 3)

    if len(lidar_data.point_cloud) < 3:
        return np.array([]).reshape(0, 3)

    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

    valid_mask = np.all(np.isfinite(points), axis=1)
    points = points[valid_mask]
    if len(points) == 0:
        return np.array([]).reshape(0, 3)

    distances = np.linalg.norm(points, axis=1)
    range_mask = (distances >= self.lidar_min_range) & (distances <= self.lidar_max_range)
    points = points[range_mask]
    if len(points) == 0:
        return np.array([]).reshape(0, 3)

    qw, qx, qy, qz = self._get_orientation_quaternion()
    x, y, z = self.get_position()

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
    ], dtype=np.float64)

    world_pts = (R @ points.astype(np.float64).T).T + np.array([x, y, z])

    return world_pts.astype(np.float32)


def get_distance_in_direction(self, target_x, target_y):
    """Get minimum lidar distance in direction of (target_x, target_y)."""
    x, y, _ = self.get_position()
    dx = target_x - x
    dy = target_y - y

    if abs(dx) < 0.01 and abs(dy) < 0.01:
        return float('inf')

    target_angle_world = math.atan2(dy, dx)

    for attempt in range(2):
        try:
            lidar_data = self.client.getLidarData()
            break
        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("getLidarData_distance")
                continue
            return float('inf')

    if len(lidar_data.point_cloud) < 3:
        return float('inf')

    points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)
    yaw = self.get_yaw()
    body_angle = target_angle_world - yaw
    point_angles = np.arctan2(points[:, 1], points[:, 0])
    angle_diff = np.abs(np.arctan2(
        np.sin(point_angles - body_angle),
        np.cos(point_angles - body_angle)
    ))

    cone_half_angle = math.radians(20.0)
    cone_mask = angle_diff < cone_half_angle
    cone_points = points[cone_mask]

    if len(cone_points) == 0:
        return float('inf')

    distances = np.linalg.norm(cone_points[:, :2], axis=1)
    return float(np.min(distances))
