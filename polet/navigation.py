"""
Navigation: navigate_to_cell, navigate_to_world.
"""

import math

from polet.constants import WALL


def _adaptive_speed(self, wall_dist):
    """Calculate flight speed based on proximity to nearest obstacle."""
    slow_zone = max(self.safe_distance * 5.0, 4.0)
    min_speed = 0.3

    if wall_dist >= slow_zone:
        return self.speed
    if wall_dist <= self.safe_distance:
        return min_speed

    t = (wall_dist - self.safe_distance) / (slow_zone - self.safe_distance)
    return min_speed + t * (self.speed - min_speed)


def navigate_to_cell(self, target_gx, target_gy, skip_yaw=False):
    """Move drone to grid cell center with adaptive speed."""
    target_x, target_y = self.grid_to_world(target_gx, target_gy)
    x, y, z = self.get_position()

    dist_to_target = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)

    if dist_to_target < 0.3:
        return True

    wall_dist = self.get_distance_in_direction(target_x, target_y)

    if wall_dist < self.safe_distance:
        self.log(f"  [NAV] Wall detected at {wall_dist:.2f}m (< {self.safe_distance}m). Stopping.")
        if wall_dist < float('inf'):
            angle = math.atan2(target_y - y, target_x - x)
            wall_x = x + math.cos(angle) * wall_dist
            wall_y = y + math.sin(angle) * wall_dist
            wgx, wgy = self.world_to_grid(wall_x, wall_y)
            if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
                self.grid[wgx][wgy] = WALL
        return False

    clearance = wall_dist - dist_to_target

    if clearance < self.safe_distance and wall_dist < dist_to_target + 0.5:
        if wall_dist < float('inf'):
            angle = math.atan2(target_y - y, target_x - x)
            wall_x = x + math.cos(angle) * wall_dist
            wall_y = y + math.sin(angle) * wall_dist
            wgx, wgy = self.world_to_grid(wall_x, wall_y)
            if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
                self.grid[wgx][wgy] = WALL
        return False

    actual_target_x = target_x
    actual_target_y = target_y
    if wall_dist < float('inf') and clearance < self.safe_distance:
        safe_fly_dist = max(0.0, wall_dist - self.safe_distance)
        if safe_fly_dist < 0.3:
            return False
        angle = math.atan2(target_y - y, target_x - x)
        actual_target_x = x + math.cos(angle) * safe_fly_dist
        actual_target_y = y + math.sin(angle) * safe_fly_dist

    fly_speed = self._adaptive_speed(wall_dist)

    if not skip_yaw:
        yaw_deg = math.degrees(math.atan2(actual_target_y - y,
                                           actual_target_x - x))
        try:
            task = self.client.rotateToYawAsync(yaw_deg, 10.0)
            self._join_with_timeout(task, timeout=5.0, task_name="nav_rotate")
        except Exception:
            pass

    fly_dist = math.sqrt((actual_target_x - x)**2 + (actual_target_y - y)**2)
    move_task = self.client.moveToPositionAsync(
        float(actual_target_x), float(actual_target_y),
        float(self.target_altitude), float(fly_speed)
    )
    if not self._join_with_timeout(move_task,
                                   timeout=self._move_timeout(fly_dist, fly_speed),
                                   task_name="nav_move"):
        self.log("  [NAV] Move timed out — stabilizing (NOT marking as wall)")
        self._keepalive_hover()
        self.record_trajectory()
        return False

    try:
        collision = self.client.simGetCollisionInfo()
        if collision.has_collided:
            self.log("  [COLLISION] Backing up...")
            backup_task = self.client.moveToPositionAsync(
                float(x), float(y),
                float(self.target_altitude), 1.0
            )
            self._join_with_timeout(backup_task, timeout=15.0,
                                    task_name="collision_backup")
            self.grid[target_gx][target_gy] = WALL
            self.record_trajectory()
            return False
    except Exception:
        pass

    self.record_trajectory()
    self._wait_for_stable(timeout=1.0, vel_threshold=0.2)
    return True


def navigate_to_world(self, target_x, target_y):
    """Navigate to a world coordinate using grid-based BFS pathfinding."""
    x, y, _ = self.get_position()
    start_gx, start_gy = self.world_to_grid(x, y)
    target_gx, target_gy = self.world_to_grid(target_x, target_y)

    if self.grid[target_gx][target_gy] == WALL:
        found = False
        for r in range(1, 6):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = target_gx + dx, target_gy + dy
                    if (0 <= nx < self.grid_dim_x and 0 <= ny < self.grid_dim_y
                            and self.grid[nx][ny] != WALL):
                        target_gx, target_gy = nx, ny
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return False

    if start_gx == target_gx and start_gy == target_gy:
        return True

    if self._grid_line_clear(start_gx, start_gy, target_gx, target_gy):
        wx, wy = self.grid_to_world(target_gx, target_gy)
        success = self.navigate_to_cell(target_gx, target_gy, skip_yaw=True)
        if success:
            self.update_grid_from_lidar(collect_for_ply=False)
            self.mark_cells_visited()
            return True

    self._keepalive_hover()
    path = self.bfs_path_to(start_gx, start_gy, target_gx, target_gy)
    if path is None:
        return False

    path = self._simplify_path(path)

    self.visualize_current_target(
        *self.grid_to_world(target_gx, target_gy),
        bfs_path=path
    )

    for i, (gx, gy) in enumerate(path[1:], 1):
        if not self.is_running_fn():
            return False

        success = self.navigate_to_cell(gx, gy, skip_yaw=True)
        if not success:
            self.update_grid_from_lidar(collect_for_ply=False)
            self.mark_cells_visited()
            return False

        self.update_grid_from_lidar(collect_for_ply=False)
        self.mark_cells_visited()

    self._wait_for_stable(timeout=1.5, vel_threshold=0.15)
    self.update_grid_from_lidar(collect_for_ply=False)
    self.mark_cells_visited()
    return True
