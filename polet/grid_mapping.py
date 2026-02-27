"""
Occupancy grid update from lidar.
"""

import math
import numpy as np

from polet.constants import UNEXPLORED, EXPLORED, WALL


def update_grid_from_lidar(self, collect_for_ply=True):
    """Update occupancy grid from lidar scan."""
    x, y, z = self.get_position()
    drone_gx, drone_gy = self.world_to_grid(x, y)

    for dgx in range(-1, 2):
        for dgy in range(-1, 2):
            ngx = drone_gx + dgx
            ngy = drone_gy + dgy
            if (0 <= ngx < self.grid_dim_x and 0 <= ngy < self.grid_dim_y
                    and self.grid[ngx][ngy] == UNEXPLORED):
                cx, cy = self.grid_to_world(ngx, ngy)
                dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
                if dist < self.cell_size * 1.5:
                    self.grid[ngx][ngy] = EXPLORED

    world_pts = self.get_lidar_points_world()

    if len(world_pts) > 0 and self.ply_output_path is not None and collect_for_ply:
        ply_pts = world_pts.copy()

        z_drone = z
        h_min = self.ply_height_min
        h_max = self.ply_height_max
        if h_min is None:
            h_min = z_drone - 8.0
        if h_max is None:
            h_max = z_drone + 3.0

        height_mask = (ply_pts[:, 2] >= h_min) & (ply_pts[:, 2] <= h_max)
        ply_pts = ply_pts[height_mask]

        if len(ply_pts) > 0:
            self.all_lidar_points.append(ply_pts)

    z_wall_tolerance = 1.0

    for pt in world_pts:
        wx, wy, wz = pt[0], pt[1], pt[2]
        wgx, wgy = self.world_to_grid(wx, wy)
        if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
            if abs(wz - z) <= z_wall_tolerance:
                if not self.visited[wgx][wgy]:
                    self.grid[wgx][wgy] = WALL
            self._mark_ray_explored(drone_gx, drone_gy, wgx, wgy)


def _mark_ray_explored(self, x0, y0, x1, y1):
    """Mark cells along ray as EXPLORED (Bresenham). Skips endpoint."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cx, cy = x0, y0

    while True:
        if cx == x1 and cy == y1:
            break
        if (0 <= cx < self.grid_dim_x and 0 <= cy < self.grid_dim_y
                and self.grid[cx][cy] == UNEXPLORED):
            self.grid[cx][cy] = EXPLORED
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy
        if (abs(cx - x0) > max(self.grid_dim_x, self.grid_dim_y) or
                abs(cy - y0) > max(self.grid_dim_x, self.grid_dim_y)):
            break
