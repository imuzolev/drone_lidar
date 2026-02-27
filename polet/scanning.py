"""
360° scanning (full and adaptive).
"""

import time
import numpy as np

from polet.constants import UNEXPLORED


def scan_full(self):
    """Rotate 360° in steps, collecting lidar at each angle."""
    angle_step = 360.0 / self.scan_rotations
    for i in range(self.scan_rotations):
        if not self.is_running_fn():
            break
        yaw = i * angle_step
        try:
            task = self.client.rotateToYawAsync(yaw, 15.0)
            self._join_with_timeout(task, timeout=12.0, task_name="scan_rotate")
        except Exception:
            pass
        self._wait_for_stable(timeout=2.0, vel_threshold=self.scan_stable_vel_threshold)
        if self.scan_pause_after_stable > 0:
            time.sleep(self.scan_pause_after_stable)
        self.update_grid_from_lidar()

    self.mark_cells_visited()
    self.record_trajectory()


def mark_cells_visited(self):
    """Mark all grid cells within visit_radius of current position."""
    x, y, _ = self.get_position()
    radius_cells = int(self.visit_radius / self.cell_size) + 1
    drone_gx, drone_gy = self.world_to_grid(x, y)

    for dgx in range(-radius_cells, radius_cells + 1):
        for dgy in range(-radius_cells, radius_cells + 1):
            ngx = drone_gx + dgx
            ngy = drone_gy + dgy
            if 0 <= ngx < self.grid_dim_x and 0 <= ngy < self.grid_dim_y:
                cx, cy = self.grid_to_world(ngx, ngy)
                dist = (cx - x) ** 2 + (cy - y) ** 2
                if dist <= self.visit_radius ** 2:
                    self.visited[ngx][ngy] = True


def _unexplored_ratio_at(self, gx, gy):
    """Ratio of UNEXPLORED cells within visit_radius of a grid position."""
    r = int(self.visit_radius / self.cell_size) + 2
    min_gx = max(0, gx - r)
    max_gx = min(self.grid_dim_x - 1, gx + r)
    min_gy = max(0, gy - r)
    max_gy = min(self.grid_dim_y - 1, gy + r)
    subgrid = self.grid[min_gx:max_gx + 1, min_gy:max_gy + 1]
    total = subgrid.size
    unexplored = int(np.sum(subgrid == UNEXPLORED))
    return unexplored / max(1, total)


def scan_adaptive(self):
    """Adaptive 360° scan: full rotation in unmapped areas, reduced in well-mapped areas."""
    if not self.use_adaptive_scan:
        rotations = self.scan_rotations
    else:
        x, y, _ = self.get_position()
        gx, gy = self.world_to_grid(x, y)
        ratio = self._unexplored_ratio_at(gx, gy)
        base_rotations = self.scan_rotations if not self.lightweight_scan_mode else min(self.scan_rotations, 8)

        if ratio > 0.15:
            rotations = base_rotations
        elif ratio > 0.05:
            rotations = max(6, base_rotations * 2 // 3)
        else:
            rotations = max(4, base_rotations // 2)

    angle_step = 360.0 / rotations
    for i in range(rotations):
        if not self.is_running_fn():
            break
        yaw = i * angle_step
        try:
            task = self.client.rotateToYawAsync(yaw, 15.0)
            self._join_with_timeout(task, timeout=12.0, task_name="adaptive_rotate")
        except Exception:
            pass
        self._wait_for_stable(timeout=2.0, vel_threshold=self.scan_stable_vel_threshold)
        if self.scan_pause_after_stable > 0:
            time.sleep(self.scan_pause_after_stable)
        self.update_grid_from_lidar()

    self.mark_cells_visited()
    self.record_trajectory()
