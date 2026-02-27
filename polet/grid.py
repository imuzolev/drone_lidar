"""
Grid configuration, coordinate conversion, auto room detection.
"""

import time
import math
import numpy as np

from polet.constants import UNEXPLORED, EXPLORED, WALL


def _configure_grid_rect(self, size_x, size_y, center_x, center_y):
    """Initialize/reinitialize rectangular grid bounds in world coordinates."""
    self.room_size_x = max(float(size_x), self.cell_size * 3.0)
    self.room_size_y = max(float(size_y), self.cell_size * 3.0)
    self.room_size = max(self.room_size_x, self.room_size_y)
    self.grid_dim_x = max(3, int(math.ceil(self.room_size_x / self.cell_size)))
    self.grid_dim_y = max(3, int(math.ceil(self.room_size_y / self.cell_size)))
    self.grid_dim = max(self.grid_dim_x, self.grid_dim_y)  # compat
    self.grid = np.zeros((self.grid_dim_x, self.grid_dim_y), dtype=int)
    self.visited = np.zeros((self.grid_dim_x, self.grid_dim_y), dtype=bool)

    self.world_min_x = float(center_x) - self.room_size_x / 2.0
    self.world_min_y = float(center_y) - self.room_size_y / 2.0


def _auto_configure_room_from_lidar(self):
    """Estimate room bounds from a 360° lidar scan and rebuild grid.
    Uses actual min/max of lidar XY hits (range-filtered) with
    IQR-based outlier rejection so that sparse far-wall points
    are kept while truly errant points are discarded.
    Supports rectangular rooms (separate X / Y sizing)."""
    all_pts = []
    angle_step = 360.0 / max(4, self.scan_rotations)
    for i in range(max(4, self.scan_rotations)):
        yaw = i * angle_step
        try:
            task = self.client.rotateToYawAsync(yaw, 15.0)
            self._join_with_timeout(task, timeout=12.0, task_name="auto_room_rotate")
        except Exception:
            pass
        self._wait_for_stable(timeout=2.0, vel_threshold=self.scan_stable_vel_threshold)
        time.sleep(max(0.1, self.scan_pause_after_stable))
        pts = self.get_lidar_points_world()
        if len(pts) > 0:
            all_pts.append(pts)

    if not all_pts:
        self.log("[AUTO ROOM] Lidar returned no points. Keeping default room size.")
        return

    world_pts = np.concatenate(all_pts, axis=0)

    for axis in range(2):
        vals = world_pts[:, axis]
        q1 = np.percentile(vals, 25)
        q3 = np.percentile(vals, 75)
        iqr = q3 - q1
        fence = max(iqr * 2.0, self.cell_size * 3.0)
        mask = (vals >= q1 - fence) & (vals <= q3 + fence)
        world_pts = world_pts[mask]
        if len(world_pts) == 0:
            self.log("[AUTO ROOM] All points filtered as outliers. "
                     "Keeping default room size.")
            return

    x_lo = float(world_pts[:, 0].min())
    x_hi = float(world_pts[:, 0].max())
    y_lo = float(world_pts[:, 1].min())
    y_hi = float(world_pts[:, 1].max())
    span_x = x_hi - x_lo
    span_y = y_hi - y_lo

    self.log(f"[AUTO ROOM] Lidar bounds X=[{x_lo:.1f}, {x_hi:.1f}] "
             f"({span_x:.1f}m)  Y=[{y_lo:.1f}, {y_hi:.1f}] ({span_y:.1f}m)")

    pad = self.auto_room_padding
    size_x = span_x + 2.0 * pad
    size_y = span_y + 2.0 * pad

    if size_x < self.cell_size * 3.0 or size_y < self.cell_size * 3.0:
        self.log("[AUTO ROOM] Detected area too small. Keeping default room size.")
        return

    center_x = (x_lo + x_hi) / 2.0
    center_y = (y_lo + y_hi) / 2.0
    old_size = self.room_size
    old_dim_x = getattr(self, 'grid_dim_x', self.grid_dim)
    old_dim_y = getattr(self, 'grid_dim_y', self.grid_dim)
    self._configure_grid_rect(size_x, size_y,
                              center_x=center_x, center_y=center_y)

    self.log(f"[AUTO ROOM] Center: ({center_x:.1f}, {center_y:.1f})")
    self.log(f"[AUTO ROOM] Grid: {old_size:.1f}m/{old_dim_x}x{old_dim_y} -> "
             f"{self.room_size_x:.1f}x{self.room_size_y:.1f}m / "
             f"{self.grid_dim_x}x{self.grid_dim_y}")


def _recalibrate_grid_from_walls(self):
    """Recalibrate grid center using discovered WALL cells."""
    wall_positions = np.argwhere(self.grid == WALL)
    if len(wall_positions) < 4:
        self.log("[RECALIBRATE] Not enough wall data to recalibrate.")
        return

    wall_world_x = self.world_min_x + wall_positions[:, 0] * self.cell_size + self.cell_size / 2
    wall_world_y = self.world_min_y + wall_positions[:, 1] * self.cell_size + self.cell_size / 2

    wx_lo = float(wall_world_x.min())
    wx_hi = float(wall_world_x.max())
    wy_lo = float(wall_world_y.min())
    wy_hi = float(wall_world_y.max())

    actual_center_x = (wx_lo + wx_hi) / 2.0
    actual_center_y = (wy_lo + wy_hi) / 2.0

    current_center_x = self.world_min_x + self.room_size_x / 2.0
    current_center_y = self.world_min_y + self.room_size_y / 2.0

    shift_x = actual_center_x - current_center_x
    shift_y = actual_center_y - current_center_y

    if abs(shift_x) < self.cell_size and abs(shift_y) < self.cell_size:
        self.log(f"[RECALIBRATE] Grid is well-centered (shift < 1 cell). "
                 f"No adjustment needed.")
        return

    self.log(f"[RECALIBRATE] Wall bounds X=[{wx_lo:.1f}, {wx_hi:.1f}] "
             f"Y=[{wy_lo:.1f}, {wy_hi:.1f}]")
    self.log(f"[RECALIBRATE] Center shift: dx={shift_x:.1f}m, dy={shift_y:.1f}m")

    wall_span_x = wx_hi - wx_lo
    wall_span_y = wy_hi - wy_lo
    pad = self.auto_room_padding
    new_size_x = max(self.room_size_x, wall_span_x + 2.0 * pad)
    new_size_y = max(self.room_size_y, wall_span_y + 2.0 * pad)

    old_grid = self.grid.copy()
    old_visited = self.visited.copy()
    old_min_x = self.world_min_x
    old_min_y = self.world_min_y
    old_dim_x = self.grid_dim_x
    old_dim_y = self.grid_dim_y

    self._configure_grid_rect(new_size_x, new_size_y,
                              center_x=actual_center_x,
                              center_y=actual_center_y)

    for ogx in range(old_dim_x):
        for ogy in range(old_dim_y):
            if old_grid[ogx][ogy] != UNEXPLORED or old_visited[ogx][ogy]:
                wx = old_min_x + ogx * self.cell_size + self.cell_size / 2
                wy = old_min_y + ogy * self.cell_size + self.cell_size / 2
                ngx, ngy = self.world_to_grid(wx, wy)
                if 0 <= ngx < self.grid_dim_x and 0 <= ngy < self.grid_dim_y:
                    if old_grid[ogx][ogy] == WALL:
                        self.grid[ngx][ngy] = WALL
                    elif old_grid[ogx][ogy] == EXPLORED:
                        if self.grid[ngx][ngy] != WALL:
                            self.grid[ngx][ngy] = EXPLORED
                    if old_visited[ogx][ogy]:
                        self.visited[ngx][ngy] = True

    self.log(f"[RECALIBRATE] Grid adjusted: "
             f"{self.room_size_x:.1f}x{self.room_size_y:.1f}m, "
             f"{self.grid_dim_x}x{self.grid_dim_y} cells, "
             f"center=({actual_center_x:.1f}, {actual_center_y:.1f})")


def world_to_grid(self, x, y):
    """Convert world coordinates to grid cell indices."""
    gx = int((x - self.world_min_x) / self.cell_size)
    gy = int((y - self.world_min_y) / self.cell_size)
    gx = max(0, min(self.grid_dim_x - 1, gx))
    gy = max(0, min(self.grid_dim_y - 1, gy))
    return gx, gy


def grid_to_world(self, gx, gy):
    """Convert grid indices to world coordinates (cell center)."""
    x = self.world_min_x + gx * self.cell_size + self.cell_size / 2
    y = self.world_min_y + gy * self.cell_size + self.cell_size / 2
    return x, y
