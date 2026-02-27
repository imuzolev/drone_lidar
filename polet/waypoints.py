"""
Coverage path generation: perimeter, interior, fill, obstacle circumnavigation.
"""

import math
from collections import deque
import numpy as np

from polet.constants import WALL


def is_location_reachable(self, gx, gy):
    """Lightweight reachability check for waypoint planning."""
    if not (0 <= gx < self.grid_dim_x and 0 <= gy < self.grid_dim_y):
        return False
    return self.grid[gx][gy] != WALL


def is_location_safe(self, gx, gy):
    """Check if grid cell is safe (far enough from known walls)."""
    if not (0 <= gx < self.grid_dim_x and 0 <= gy < self.grid_dim_y):
        return False

    if self.grid[gx][gy] == WALL:
        return False

    safe_radius_cells = int(math.ceil(self.safe_distance / self.cell_size))

    min_gx = max(0, gx - safe_radius_cells)
    max_gx = min(self.grid_dim_x - 1, gx + safe_radius_cells)
    min_gy = max(0, gy - safe_radius_cells)
    max_gy = min(self.grid_dim_y - 1, gy + safe_radius_cells)

    subgrid = self.grid[min_gx:max_gx+1, min_gy:max_gy+1]
    if not np.any(subgrid == WALL):
        return True

    tx, ty = self.grid_to_world(gx, gy)
    wall_indices = np.argwhere(subgrid == WALL)

    for w_idx in wall_indices:
        wgx = min_gx + w_idx[0]
        wgy = min_gy + w_idx[1]
        wx, wy = self.grid_to_world(wgx, wgy)
        dist = math.sqrt((wx - tx)**2 + (wy - ty)**2)
        if dist < self.safe_distance:
            return False

    return True


def _generate_perimeter_waypoints(self):
    """Generate waypoints along the room perimeter."""
    margin = max(self.safe_distance * 0.5, self.cell_size)
    min_x = self.world_min_x + margin
    max_x = self.world_min_x + self.room_size_x - margin
    min_y = self.world_min_y + margin
    max_y = self.world_min_y + self.room_size_y - margin

    spacing = self.coverage_spacing
    x, y, _ = self.get_position()

    corners = [
        (min_x, min_y), (max_x, min_y),
        (max_x, max_y), (min_x, max_y)
    ]
    nearest_idx = min(range(4), key=lambda i:
        (corners[i][0] - x)**2 + (corners[i][1] - y)**2)
    ordered = corners[nearest_idx:] + corners[:nearest_idx]

    waypoints = []
    for ci in range(4):
        c1 = ordered[ci]
        c2 = ordered[(ci + 1) % 4]
        dx = c2[0] - c1[0]
        dy = c2[1] - c1[1]
        edge_len = math.sqrt(dx * dx + dy * dy)
        n_steps = max(1, int(edge_len / spacing))
        for s in range(n_steps + 1):
            t = s / max(1, n_steps)
            wx = c1[0] + dx * t
            wy = c1[1] + dy * t
            waypoints.append((wx, wy))

    filtered = [waypoints[0]]
    for wp in waypoints[1:]:
        if math.sqrt((wp[0] - filtered[-1][0])**2 +
                     (wp[1] - filtered[-1][1])**2) > spacing * 0.4:
            filtered.append(wp)

    valid = []
    for wx, wy in filtered:
        gx, gy = self.world_to_grid(wx, wy)
        if self.is_location_reachable(gx, gy):
            valid.append((wx, wy))

    return valid


def _generate_interior_waypoints(self):
    """Generate interior boustrophedon (lawnmower) waypoints."""
    margin = max(self.safe_distance * 0.5, self.cell_size)
    interior_off = self.coverage_spacing * 0.3
    min_x = self.world_min_x + margin + interior_off
    max_x = self.world_min_x + self.room_size_x - margin - interior_off
    min_y = self.world_min_y + margin + interior_off
    max_y = self.world_min_y + self.room_size_y - margin - interior_off

    if min_x >= max_x or min_y >= max_y:
        return []

    spacing = self.coverage_spacing
    raw_waypoints = []
    forward = True

    y = min_y
    while y <= max_y:
        if forward:
            x = min_x
            while x <= max_x:
                raw_waypoints.append((x, y))
                x += spacing
            if raw_waypoints and raw_waypoints[-1][0] < max_x - 0.5:
                raw_waypoints.append((max_x, y))
        else:
            x = max_x
            while x >= min_x:
                raw_waypoints.append((x, y))
                x -= spacing
            if raw_waypoints and raw_waypoints[-1][0] > min_x + 0.5:
                raw_waypoints.append((min_x, y))
        y += spacing
        forward = not forward

    valid = []
    for wx, wy in raw_waypoints:
        gx, gy = self.world_to_grid(wx, wy)
        if self.is_location_reachable(gx, gy):
            valid.append((wx, wy))

    return valid


def generate_coverage_waypoints(self):
    """Generate full coverage waypoints: perimeter + interior lawnmower."""
    return self._generate_perimeter_waypoints() + self._generate_interior_waypoints()


def _generate_fill_waypoints(self, spacing=None):
    """Generate waypoints targeting only unvisited reachable cells."""
    if spacing is None:
        spacing = self.coverage_spacing * 0.6
    margin = max(self.safe_distance * 0.5, self.cell_size)
    min_x = self.world_min_x + margin
    max_x = self.world_min_x + self.room_size_x - margin
    min_y = self.world_min_y + margin
    max_y = self.world_min_y + self.room_size_y - margin

    if min_x >= max_x or min_y >= max_y:
        return []

    raw_waypoints = []
    forward = True

    y = min_y
    while y <= max_y:
        if forward:
            x = min_x
            while x <= max_x:
                raw_waypoints.append((x, y))
                x += spacing
            if raw_waypoints and raw_waypoints[-1][0] < max_x - 0.5:
                raw_waypoints.append((max_x, y))
        else:
            x = max_x
            while x >= min_x:
                raw_waypoints.append((x, y))
                x -= spacing
            if raw_waypoints and raw_waypoints[-1][0] > min_x + 0.5:
                raw_waypoints.append((min_x, y))
        y += spacing
        forward = not forward

    valid = []
    for wx, wy in raw_waypoints:
        gx, gy = self.world_to_grid(wx, wy)
        if (self.is_location_reachable(gx, gy) and
                not self.visited[gx][gy]):
            valid.append((wx, wy))

    return valid


def _detect_internal_obstacles(self):
    """Detect clusters of WALL cells that are internal obstacles."""
    wall_mask = (self.grid == WALL)
    visited_cells = np.zeros_like(wall_mask, dtype=bool)
    clusters = []
    border = 2

    for gx in range(self.grid_dim_x):
        for gy in range(self.grid_dim_y):
            if wall_mask[gx, gy] and not visited_cells[gx, gy]:
                cluster = []
                touches_border = False
                queue = deque([(gx, gy)])
                visited_cells[gx, gy] = True

                while queue:
                    cx, cy = queue.popleft()
                    cluster.append((cx, cy))

                    if (cx < border or cx >= self.grid_dim_x - border or
                            cy < border or cy >= self.grid_dim_y - border):
                        touches_border = True

                    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1),
                                   (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                        nx, ny = cx + dx, cy + dy
                        if (0 <= nx < self.grid_dim_x and
                                0 <= ny < self.grid_dim_y and
                                wall_mask[nx, ny] and
                                not visited_cells[nx, ny]):
                            visited_cells[nx, ny] = True
                            queue.append((nx, ny))

                if not touches_border and len(cluster) >= 1:
                    clusters.append(cluster)

    return clusters


def _obstacle_centroid_and_radius(self, cluster):
    """Compute centroid (world coords) and effective radius of a cluster."""
    coords = np.array(cluster, dtype=float)
    mean_gx = coords[:, 0].mean()
    mean_gy = coords[:, 1].mean()
    cx, cy = self.grid_to_world(int(round(mean_gx)), int(round(mean_gy)))

    max_dist = 0.0
    for gx, gy in cluster:
        wx, wy = self.grid_to_world(gx, gy)
        d = math.sqrt((wx - cx) ** 2 + (wy - cy) ** 2)
        if d > max_dist:
            max_dist = d

    return cx, cy, max_dist + self.cell_size * 0.5


def _generate_obstacle_circumnavigation_waypoints(self):
    """Generate waypoints around each internal obstacle."""
    clusters = self._detect_internal_obstacles()
    if not clusters:
        return []

    self.log(f"[OBSTACLES] Detected {len(clusters)} internal obstacle(s)")

    all_obstacle_wps = []
    orbit_angles = 8
    margin = max(self.safe_distance, self.cell_size * 2.0)

    for idx, cluster in enumerate(clusters):
        cx, cy, radius = self._obstacle_centroid_and_radius(cluster)
        orbit_dist = radius + margin
        self.log(f"[OBSTACLES] Obstacle {idx + 1}: center=({cx:.1f}, {cy:.1f}), "
                 f"radius={radius:.1f}m, orbit={orbit_dist:.1f}m")

        waypoints = []
        for ai in range(orbit_angles):
            angle = ai * (2.0 * math.pi / orbit_angles)
            wx = cx + orbit_dist * math.cos(angle)
            wy = cy + orbit_dist * math.sin(angle)

            gx, gy = self.world_to_grid(wx, wy)
            if (0 <= gx < self.grid_dim_x and 0 <= gy < self.grid_dim_y
                    and self.is_location_reachable(gx, gy)):
                waypoints.append((wx, wy))

        if waypoints:
            all_obstacle_wps.append((f"Obstacle_{idx + 1}", waypoints))
            self.log(f"[OBSTACLES] Obstacle {idx + 1}: "
                     f"{len(waypoints)}/{orbit_angles} waypoints reachable")

    return all_obstacle_wps
