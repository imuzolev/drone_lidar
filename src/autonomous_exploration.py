"""
Autonomous Full-Coverage Room Explorer for AirSim Drones.

Flies a systematic lawnmower pattern across the entire map, performs
360-degree lidar scanning at each waypoint, builds a dense point cloud,
avoids obstacles, and returns home after full coverage.

Exploration phases:
  Phase 1 - Initial 360° scan to detect room bounds and obstacles.
  Phase 2 - Lawnmower (boustrophedon) coverage flight with 360° scans.
  Phase 3 - BFS cleanup pass to visit any missed free cells.
  Phase 4 - Return to start with smooth descent and landing.

Usage:
    python autonomous_exploration.py                      # Real AirSim
    python autonomous_exploration.py --mock               # Mock mode
    python autonomous_exploration.py --auto-room-size     # Auto-detect map
    python autonomous_exploration.py --allow-mock-fallback
"""

import time
import os
import numpy as np
import math
from collections import deque
import sys
import argparse

# Try to import airsim
try:
    import airsim
except ImportError:
    airsim = None

import mock_airsim

# Grid cell states
UNEXPLORED = 0
EXPLORED = 1   # Lidar ray passed through (known free space)
WALL = 2


class AutonomousExplorer:
    """
    Full-coverage autonomous drone explorer.

    Divides the room into a fine grid, flies a lawnmower pattern to
    physically visit every reachable area, collects dense 360-degree
    lidar at each waypoint, and builds a point cloud that matches
    the real environment.
    """

    def __init__(self, client=None, use_mock=False, room_size=20.0,
                 cell_size=2.0, altitude=3.0, speed=1.5, safe_distance=3.0,
                 require_sim=True, auto_room_size=False, auto_room_padding=1.0,
                 scan_rotations=12, visit_radius=5.0, coverage_spacing=5.0):
        """
        Args:
            client:           Existing AirSim client (None => creates new).
            use_mock:         Use mock AirSim when no client provided.
            room_size:        Room dimension in meters (square room).
            cell_size:        Grid cell size in meters.
            altitude:         Flight altitude in meters (positive).
            speed:            Flight speed in m/s.
            safe_distance:    Min distance from obstacles in meters.
            require_sim:      Fail if AirSim unavailable (no silent mock).
            auto_room_size:   Detect map bounds from first lidar scan.
            auto_room_padding: Extra padding around detected walls (m).
            scan_rotations:   Number of rotation steps for 360° scan.
            visit_radius:     Radius (m) within which cells count as visited.
            coverage_spacing: Distance (m) between lawnmower rows.
        """
        # Logging
        self.log = print
        self.is_running_fn = lambda: True

        # Connection
        self.use_mock = use_mock
        self.client = client
        self.external_client = client is not None
        self.require_sim = require_sim

        if not self.external_client:
            self._connect(use_mock)

        # Grid parameters
        self.room_size = room_size
        self.cell_size = cell_size
        self.grid_dim = 0
        self.grid = np.zeros((1, 1), dtype=int)
        self.visited = np.zeros((1, 1), dtype=bool)
        self.world_min_x = 0.0
        self.world_min_y = 0.0
        self._configure_grid(room_size=room_size, center_x=0.0, center_y=0.0)

        # Flight parameters
        self.target_altitude = -abs(altitude)  # NED: negative = up
        self.safe_distance = safe_distance
        self.speed = speed
        self.auto_room_size = auto_room_size
        self.auto_room_padding = max(0.0, float(auto_room_padding))

        # Scanning parameters
        self.scan_rotations = max(4, int(scan_rotations))
        self.visit_radius = max(self.cell_size, float(visit_radius))
        self.coverage_spacing = max(self.cell_size, float(coverage_spacing))

        # State
        self.start_position = None
        self.trajectory_points = []

        # Point cloud accumulation for PLY export
        self.all_lidar_points = []
        self.ply_output_path = None
        self.ply_voxel_size = 0.05

    # ================================================================
    #  CONNECTION
    # ================================================================

    def _connect(self, use_mock):
        """Connect to AirSim or fall back to mock."""
        if not use_mock and airsim is not None:
            try:
                self.log("[CONNECT] Connecting to AirSim...")
                self.client = airsim.MultirotorClient()
                self.client.confirmConnection()
                self.log("[CONNECT] Connected to AirSim.")
                self.client.reset()
                import time as _t
                _t.sleep(1.0)
                self.client.enableApiControl(True)
                self.client.armDisarm(True)
                self.log("[CONNECT] Simulator reset and ready.")
                self.use_mock = False
                return
            except Exception as e:
                self.log(f"[ERROR] AirSim: {e}")
                if self.require_sim:
                    raise RuntimeError(
                        "Could not connect to AirSim. "
                        "Start UE/Colosseum simulator or add --allow-mock-fallback."
                    ) from e
                self.log("[CONNECT] Falling back to mock mode.")

        if not use_mock and airsim is None and self.require_sim:
            raise RuntimeError(
                "Package 'airsim' is not available in this environment. "
                "Install Colosseum AirSim Python client or use --mock."
            )

        self.use_mock = True
        self.client = mock_airsim.MultirotorClient()
        self.client.confirmConnection()

    # ================================================================
    #  GRID CONFIGURATION
    # ================================================================

    def _configure_grid(self, room_size, center_x, center_y):
        """Initialize/reinitialize square grid bounds in world coordinates."""
        self.room_size = max(float(room_size), self.cell_size * 3.0)
        self.grid_dim = max(3, int(math.ceil(self.room_size / self.cell_size)))
        self.grid = np.zeros((self.grid_dim, self.grid_dim), dtype=int)
        self.visited = np.zeros((self.grid_dim, self.grid_dim), dtype=bool)

        half = self.room_size / 2.0
        self.world_min_x = float(center_x) - half
        self.world_min_y = float(center_y) - half

    def _auto_configure_room_from_lidar(self):
        """Estimate room size from a 360° lidar scan and rebuild grid.
        Uses percentile-based bounds to exclude outlier points (distant
        objects visible through openings, etc.)."""
        # Collect points from a full rotation for better coverage
        all_pts = []
        angle_step = 360.0 / max(4, self.scan_rotations)
        for i in range(max(4, self.scan_rotations)):
            yaw = i * angle_step
            try:
                self.client.rotateToYawAsync(yaw, 0.5).join()
            except Exception:
                pass
            time.sleep(0.15)
            pts = self.get_lidar_points_world()
            if len(pts) > 0:
                all_pts.append(pts)

        if not all_pts:
            self.log("[AUTO ROOM] Lidar returned no points. Keeping default room size.")
            return

        world_pts = np.concatenate(all_pts, axis=0)

        # Use 5th/95th percentile to exclude far-away outliers
        x_lo = float(np.percentile(world_pts[:, 0], 5))
        x_hi = float(np.percentile(world_pts[:, 0], 95))
        y_lo = float(np.percentile(world_pts[:, 1], 5))
        y_hi = float(np.percentile(world_pts[:, 1], 95))
        span_x = x_hi - x_lo
        span_y = y_hi - y_lo
        detected_size = max(span_x, span_y) + 2.0 * self.auto_room_padding

        if detected_size < self.cell_size * 3.0:
            self.log("[AUTO ROOM] Detected area too small. Keeping default room size.")
            return

        center_x = (x_lo + x_hi) / 2.0
        center_y = (y_lo + y_hi) / 2.0
        old_size = self.room_size
        old_dim = self.grid_dim
        self._configure_grid(detected_size, center_x=center_x, center_y=center_y)

        self.log(f"[AUTO ROOM] Lidar bounds (p5-p95) X=[{x_lo:.1f},{x_hi:.1f}] "
                 f"Y=[{y_lo:.1f},{y_hi:.1f}]")
        self.log(f"[AUTO ROOM] Grid: {old_size:.1f}m/{old_dim}x{old_dim} -> "
                 f"{self.room_size:.1f}m/{self.grid_dim}x{self.grid_dim}")

    # ================================================================
    #  COORDINATE CONVERSION
    # ================================================================

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid cell indices."""
        gx = int((x - self.world_min_x) / self.cell_size)
        gy = int((y - self.world_min_y) / self.cell_size)
        gx = max(0, min(self.grid_dim - 1, gx))
        gy = max(0, min(self.grid_dim - 1, gy))
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates (cell center)."""
        x = self.world_min_x + gx * self.cell_size + self.cell_size / 2
        y = self.world_min_y + gy * self.cell_size + self.cell_size / 2
        return x, y

    # ================================================================
    #  DRONE STATE
    # ================================================================

    def get_position(self):
        """Get current position as (x, y, z) tuple."""
        state = self.client.getMultirotorState()
        p = state.kinematics_estimated.position
        return p.x_val, p.y_val, p.z_val

    def get_yaw(self):
        """Get current yaw angle in radians."""
        state = self.client.getMultirotorState()
        o = state.kinematics_estimated.orientation
        siny = 2.0 * (o.w_val * o.z_val + o.x_val * o.y_val)
        cosy = 1.0 - 2.0 * (o.y_val ** 2 + o.z_val ** 2)
        return math.atan2(siny, cosy)

    # ================================================================
    #  TRAJECTORY DRAWING
    # ================================================================

    def record_trajectory(self):
        """Record position and draw red trajectory line in UE5."""
        x, y, z = self.get_position()

        if not self.use_mock and airsim is not None:
            point = airsim.Vector3r(x, y, z)
            self.trajectory_points.append(point)
            if len(self.trajectory_points) >= 2:
                try:
                    self.client.simPlotLineStrip(
                        self.trajectory_points,
                        color_rgba=[1.0, 0.0, 0.0, 1.0],
                        thickness=8.0,
                        duration=300.0,
                        is_persistent=True
                    )
                except Exception:
                    pass
        else:
            self.trajectory_points.append((x, y, z))

    # ================================================================
    #  LIDAR PROCESSING
    # ================================================================

    def get_lidar_points_world(self):
        """Get lidar point cloud converted to world coordinates.
        Returns numpy array (N, 3)."""
        try:
            lidar_data = self.client.getLidarData()
        except Exception:
            return np.array([]).reshape(0, 3)

        if len(lidar_data.point_cloud) < 3:
            return np.array([]).reshape(0, 3)

        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

        yaw = self.get_yaw()
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        x, y, z = self.get_position()

        world_pts = np.zeros_like(points)
        world_pts[:, 0] = x + points[:, 0] * cos_y - points[:, 1] * sin_y
        world_pts[:, 1] = y + points[:, 0] * sin_y + points[:, 1] * cos_y
        world_pts[:, 2] = z + points[:, 2]

        return world_pts

    def get_distance_in_direction(self, target_x, target_y):
        """Get minimum lidar distance in direction of (target_x, target_y).
        Returns distance in meters, or inf."""
        x, y, _ = self.get_position()
        dx = target_x - x
        dy = target_y - y

        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return float('inf')

        target_angle_world = math.atan2(dy, dx)

        try:
            lidar_data = self.client.getLidarData()
        except Exception:
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

        cone_half_angle = math.radians(22.5)
        cone_mask = angle_diff < cone_half_angle
        cone_points = points[cone_mask]

        if len(cone_points) == 0:
            return float('inf')

        distances = np.linalg.norm(cone_points[:, :2], axis=1)
        return float(np.min(distances))

    # ================================================================
    #  GRID MAPPING
    # ================================================================

    def update_grid_from_lidar(self):
        """Update occupancy grid from lidar scan.
        Marks WALL at lidar endpoints, EXPLORED along rays (free space).
        Also accumulates points for PLY export."""
        x, y, _ = self.get_position()
        drone_gx, drone_gy = self.world_to_grid(x, y)

        # Mark cells near drone as free
        for dgx in range(-1, 2):
            for dgy in range(-1, 2):
                ngx = drone_gx + dgx
                ngy = drone_gy + dgy
                if (0 <= ngx < self.grid_dim and 0 <= ngy < self.grid_dim
                        and self.grid[ngx][ngy] == UNEXPLORED):
                    cx, cy = self.grid_to_world(ngx, ngy)
                    dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
                    if dist < self.cell_size * 1.5:
                        self.grid[ngx][ngy] = EXPLORED

        world_pts = self.get_lidar_points_world()

        # Accumulate for PLY
        if len(world_pts) > 0 and self.ply_output_path is not None:
            self.all_lidar_points.append(world_pts.copy())

        for pt in world_pts:
            wx, wy = pt[0], pt[1]
            wgx, wgy = self.world_to_grid(wx, wy)
            if 0 <= wgx < self.grid_dim and 0 <= wgy < self.grid_dim:
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
            if (0 <= cx < self.grid_dim and 0 <= cy < self.grid_dim
                    and self.grid[cx][cy] == UNEXPLORED):
                self.grid[cx][cy] = EXPLORED
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy
            if abs(cx - x0) > self.grid_dim or abs(cy - y0) > self.grid_dim:
                break

    # ================================================================
    #  360° SCANNING
    # ================================================================

    def scan_full(self):
        """Rotate 360° in steps, collecting lidar at each angle.
        Marks nearby cells as visited after scanning."""
        angle_step = 360.0 / self.scan_rotations
        for i in range(self.scan_rotations):
            if not self.is_running_fn():
                break
            yaw = i * angle_step
            try:
                self.client.rotateToYawAsync(yaw, 0.5).join()
            except Exception:
                pass
            time.sleep(0.15)
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
                if 0 <= ngx < self.grid_dim and 0 <= ngy < self.grid_dim:
                    cx, cy = self.grid_to_world(ngx, ngy)
                    dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
                    if dist <= self.visit_radius:
                        self.visited[ngx][ngy] = True

    # ================================================================
    #  COVERAGE PATH GENERATION (LAWNMOWER)
    # ================================================================

    def generate_coverage_waypoints(self):
        """Generate lawnmower/boustrophedon waypoints for systematic map coverage.
        Returns list of (world_x, world_y) tuples."""
        margin = self.safe_distance
        min_x = self.world_min_x + margin
        max_x = self.world_min_x + self.room_size - margin
        min_y = self.world_min_y + margin
        max_y = self.world_min_y + self.room_size - margin

        spacing = self.coverage_spacing
        waypoints = []
        forward = True

        y = min_y
        while y <= max_y:
            if forward:
                x = min_x
                while x <= max_x:
                    waypoints.append((x, y))
                    x += spacing
            else:
                x = max_x
                while x >= min_x:
                    waypoints.append((x, y))
                    x -= spacing
            y += spacing
            forward = not forward

        return waypoints

    # ================================================================
    #  PATH PLANNING
    # ================================================================

    def bfs_path_to(self, start_gx, start_gy, target_gx, target_gy):
        """BFS from (start) to (target) through non-WALL cells.
        Returns list of (gx, gy) or None."""
        start = (start_gx, start_gy)
        target = (target_gx, target_gy)

        if start == target:
            return [start]

        queue = deque([start])
        seen = {start}
        parent = {start: None}

        while queue:
            current = queue.popleft()
            if current == target:
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = parent[node]
                path.reverse()
                return path

            gx, gy = current
            for dgx, dgy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = gx + dgx, gy + dgy
                nb = (nx, ny)
                if (0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim
                        and nb not in seen
                        and self.grid[nx][ny] != WALL):
                    seen.add(nb)
                    parent[nb] = current
                    queue.append(nb)

        return None

    def find_path_to_nearest_unvisited(self):
        """BFS to nearest EXPLORED cell that has not been visited.
        Returns path as list of (gx, gy) or None."""
        x, y, _ = self.get_position()
        start = self.world_to_grid(x, y)

        queue = deque([start])
        seen = {start}
        parent = {start: None}

        while queue:
            current = queue.popleft()
            gx, gy = current

            # Target: free cell that hasn't been visited
            if self.grid[gx][gy] == EXPLORED and not self.visited[gx][gy]:
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = parent[node]
                path.reverse()
                return path

            for dgx, dgy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = gx + dgx, gy + dgy
                nb = (nx, ny)
                if (0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim
                        and nb not in seen
                        and self.grid[nx][ny] != WALL):
                    seen.add(nb)
                    parent[nb] = current
                    queue.append(nb)

        return None

    def _grid_line_clear(self, gx0, gy0, gx1, gy1):
        """Check if a straight line between two grid cells crosses any WALL cell.
        Uses Bresenham. Returns True if clear."""
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        cx, cy = gx0, gy0

        while True:
            if 0 <= cx < self.grid_dim and 0 <= cy < self.grid_dim:
                if self.grid[cx][cy] == WALL:
                    return False
            else:
                return False  # Out of bounds = not clear

            if cx == gx1 and cy == gy1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                cx += sx
            if e2 < dx:
                err += dx
                cy += sy

            if abs(cx - gx0) > self.grid_dim or abs(cy - gy0) > self.grid_dim:
                return False

        return True

    # ================================================================
    #  NAVIGATION
    # ================================================================

    def _adaptive_speed(self, wall_dist):
        """Calculate flight speed based on proximity to nearest obstacle.
        Returns speed in m/s — full speed when far, crawl speed when close.

        Speed zones (distances are from obstacle):
          > slow_zone   : full speed
          slow_zone     : linear ramp-down from full to min speed
          < safe_dist   : blocked (should not fly)
        """
        slow_zone = max(self.safe_distance * 5.0, 4.0)  # start slowing ~4m out
        min_speed = 0.3  # crawl speed near walls

        if wall_dist >= slow_zone:
            return self.speed
        if wall_dist <= self.safe_distance:
            return min_speed

        # Linear interpolation
        t = (wall_dist - self.safe_distance) / (slow_zone - self.safe_distance)
        return min_speed + t * (self.speed - min_speed)

    def navigate_to_cell(self, target_gx, target_gy):
        """Move drone to grid cell center with adaptive speed.
        Flies as close to obstacles as possible, slowing down near walls.
        Returns True if reached, False if truly blocked."""
        target_x, target_y = self.grid_to_world(target_gx, target_gy)
        x, y, z = self.get_position()

        dist_to_target = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)

        if dist_to_target < 0.3:
            return True

        # Lidar obstacle check in direction of target
        wall_dist = self.get_distance_in_direction(target_x, target_y)
        clearance = wall_dist - dist_to_target  # space left after reaching target

        # Hard block: we would overshoot INTO the wall
        if clearance < self.safe_distance and wall_dist < dist_to_target + 0.5:
            # Mark wall cell on the map
            if wall_dist < float('inf'):
                angle = math.atan2(target_y - y, target_x - x)
                wall_x = x + math.cos(angle) * wall_dist
                wall_y = y + math.sin(angle) * wall_dist
                wgx, wgy = self.world_to_grid(wall_x, wall_y)
                if 0 <= wgx < self.grid_dim and 0 <= wgy < self.grid_dim:
                    self.grid[wgx][wgy] = WALL
            if clearance < -0.3:
                self.grid[target_gx][target_gy] = WALL
            return False

        # If there's an obstacle ahead but we still have room, fly closer
        # but cap destination before the wall
        actual_target_x = target_x
        actual_target_y = target_y
        if wall_dist < float('inf') and clearance < self.safe_distance:
            # Move as close as possible: stop safe_distance before the wall
            safe_fly_dist = max(0.0, wall_dist - self.safe_distance)
            if safe_fly_dist < 0.3:
                return False  # too close already
            angle = math.atan2(target_y - y, target_x - x)
            actual_target_x = x + math.cos(angle) * safe_fly_dist
            actual_target_y = y + math.sin(angle) * safe_fly_dist

        # Calculate adaptive speed
        fly_speed = self._adaptive_speed(wall_dist)

        # Rotate toward target
        yaw_deg = math.degrees(math.atan2(actual_target_y - y,
                                           actual_target_x - x))
        try:
            self.client.rotateToYawAsync(yaw_deg, 0.5).join()
        except Exception:
            pass

        # Fly at adaptive speed
        self.client.moveToPositionAsync(
            float(actual_target_x), float(actual_target_y),
            float(self.target_altitude), float(fly_speed)
        ).join()

        # Post-flight collision check
        try:
            collision = self.client.simGetCollisionInfo()
            if collision.has_collided:
                self.log("  [COLLISION] Backing up...")
                self.client.moveToPositionAsync(
                    float(x), float(y),
                    float(self.target_altitude), 1.0
                ).join()
                self.grid[target_gx][target_gy] = WALL
                self.record_trajectory()
                return False
        except Exception:
            pass

        self.record_trajectory()
        return True

    def navigate_to_world(self, target_x, target_y):
        """Navigate to a world coordinate using grid-based BFS pathfinding.
        Collects lidar along the way. Returns True if reached."""
        x, y, _ = self.get_position()
        start_gx, start_gy = self.world_to_grid(x, y)
        target_gx, target_gy = self.world_to_grid(target_x, target_y)

        # Skip if target is a wall; find nearest free cell
        if self.grid[target_gx][target_gy] == WALL:
            found = False
            for r in range(1, 6):
                for dx in range(-r, r + 1):
                    for dy in range(-r, r + 1):
                        nx, ny = target_gx + dx, target_gy + dy
                        if (0 <= nx < self.grid_dim and 0 <= ny < self.grid_dim
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

        # Try direct flight if grid line is clear
        if self._grid_line_clear(start_gx, start_gy, target_gx, target_gy):
            wx, wy = self.grid_to_world(target_gx, target_gy)
            success = self.navigate_to_cell(target_gx, target_gy)
            if success:
                self.update_grid_from_lidar()
                self.mark_cells_visited()
                return True

        # BFS pathfinding
        path = self.bfs_path_to(start_gx, start_gy, target_gx, target_gy)
        if path is None:
            return False

        # Follow path, scanning periodically
        scan_interval = max(1, int(self.coverage_spacing / self.cell_size))
        for i, (gx, gy) in enumerate(path[1:], 1):
            if not self.is_running_fn():
                return False

            success = self.navigate_to_cell(gx, gy)
            if not success:
                # Re-scan and try to replan
                self.update_grid_from_lidar()
                self.mark_cells_visited()
                return False

            # Collect lidar along the path
            if i % scan_interval == 0:
                self.update_grid_from_lidar()
                self.mark_cells_visited()

        # Final lidar collection at destination
        self.update_grid_from_lidar()
        self.mark_cells_visited()
        return True

    # ================================================================
    #  FLIGHT LIFECYCLE
    # ================================================================

    def takeoff(self):
        """Take off and ascend to exploration altitude."""
        self.log("[FLIGHT] Taking off...")
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()
        self.start_position = state.kinematics_estimated.position
        self.log(f"[FLIGHT] Start: ({self.start_position.x_val:.1f}, "
                 f"{self.start_position.y_val:.1f})")

        self.record_trajectory()

        self.log(f"[FLIGHT] Ascending to {abs(self.target_altitude):.0f}m...")
        self.client.moveToPositionAsync(
            self.start_position.x_val,
            self.start_position.y_val,
            self.target_altitude,
            2.0
        ).join()
        self.record_trajectory()
        self.log("[FLIGHT] At target altitude. Ready to explore.")

    def return_home_smooth(self):
        """Smoothly return to start position with gradual descent."""
        if not self.start_position:
            self.log("[ERROR] Start position not recorded!")
            return

        sx = self.start_position.x_val
        sy = self.start_position.y_val

        self.log("[FLIGHT] === RETURNING HOME ===")

        self.log(f"[FLIGHT] Flying to start ({sx:.1f}, {sy:.1f})...")
        self.client.moveToPositionAsync(
            sx, sy, self.target_altitude, self.speed
        ).join()
        self.record_trajectory()

        self.log("[FLIGHT] Hovering...")
        try:
            self.client.hoverAsync()
        except Exception:
            pass
        time.sleep(0.5)

        # Gradual descent
        alt = abs(self.target_altitude)
        current_alt = alt
        while current_alt > 0.8:
            current_alt *= 0.5
            self.log(f"[FLIGHT] Descending to {current_alt:.1f}m...")
            self.client.moveToPositionAsync(
                sx, sy, float(-current_alt), 0.8
            ).join()
            time.sleep(0.2)

        self.log("[FLIGHT] Descending to 0.3m...")
        self.client.moveToPositionAsync(sx, sy, -0.3, 0.5).join()
        time.sleep(0.2)
        self.record_trajectory()

        # Land with timeout
        self.log("[FLIGHT] Landing...")
        try:
            self.client.landAsync().join()
        except Exception as e:
            self.log(f"[FLIGHT] Landing exception (non-critical): {e}")
        self.record_trajectory()
        self.log("[FLIGHT] Landed successfully.")

    # ================================================================
    #  GRID VISUALIZATION
    # ================================================================

    def print_grid(self, compact=False):
        """Print the occupancy grid with visited overlay."""
        x, y, _ = self.get_position()
        drone_gx, drone_gy = self.world_to_grid(x, y)

        explored_count = int(np.sum(self.grid == EXPLORED))
        wall_count = int(np.sum(self.grid == WALL))
        unexplored_count = int(np.sum(self.grid == UNEXPLORED))
        visited_free = int(np.sum(self.visited & (self.grid == EXPLORED)))
        total = self.grid_dim * self.grid_dim

        self.log("")
        self.log("=" * 60)
        self.log(f"  GRID  ({self.grid_dim}x{self.grid_dim}, "
                 f"cell={self.cell_size}m, room={self.room_size:.1f}m)")
        self.log(f"  Free: {explored_count}  Walls: {wall_count}  "
                 f"Unexplored: {unexplored_count}")
        self.log(f"  Visited: {visited_free}/{explored_count} "
                 f"({100 * visited_free / max(1, explored_count):.0f}% coverage)")
        self.log(f"  D=drone  +=visited  .=free  ?=unknown  #=wall")
        self.log("=" * 60)

        if compact and self.grid_dim > 50:
            self.log("  (grid too large for console, showing stats only)")
            self.log("=" * 60)
            return

        header = "    " + "".join(f"{i:>2}" for i in range(self.grid_dim))
        self.log(header)

        for gx in range(self.grid_dim - 1, -1, -1):
            row = f" {gx:>2} "
            for gy in range(self.grid_dim):
                if gx == drone_gx and gy == drone_gy:
                    row += " D"
                elif self.grid[gx][gy] == WALL:
                    row += " #"
                elif self.visited[gx][gy]:
                    row += " +"
                elif self.grid[gx][gy] == EXPLORED:
                    row += " ."
                else:
                    row += " ?"
            self.log(row)

        self.log("=" * 60)

    # ================================================================
    #  POINT CLOUD EXPORT (PLY)
    # ================================================================

    def _voxel_downsample(self, points, voxel_size):
        """Voxel grid downsampling."""
        if len(points) == 0:
            return points
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_idx]

    def _height_to_rgb(self, z_values):
        """Map Z values to blue-green-yellow-red colormap."""
        if len(z_values) == 0:
            return np.zeros((0, 3), dtype=np.uint8)
        z_min, z_max = z_values.min(), z_values.max()
        if z_max - z_min < 0.01:
            t = np.full_like(z_values, 0.5)
        else:
            t = (z_values - z_min) / (z_max - z_min)
            t = 1.0 - t
        r = np.clip(np.where(t < 0.5, 0.0, (t - 0.5) * 2.0), 0, 1)
        g = np.clip(np.where(t < 0.5, t * 2.0, 2.0 - t * 2.0), 0, 1)
        b = np.clip(np.where(t < 0.5, 1.0 - t * 2.0, 0.0), 0, 1)
        rgb = np.stack([r, g, b], axis=1)
        return (rgb * 255).astype(np.uint8)

    def save_point_cloud_ply(self, filepath=None):
        """Save accumulated lidar point cloud to PLY."""
        filepath = filepath or self.ply_output_path or "point_cloud.ply"

        if not self.all_lidar_points:
            self.log("[PLY] No lidar points collected.")
            return None

        self.log("[PLY] Merging point cloud from all scans...")
        all_pts = np.concatenate(self.all_lidar_points, axis=0)
        total_raw = len(all_pts)
        self.log(f"[PLY] Raw points: {total_raw:,} "
                 f"(from {len(self.all_lidar_points)} scans)")

        if self.ply_voxel_size > 0:
            all_pts = self._voxel_downsample(all_pts, self.ply_voxel_size)
            self.log(f"[PLY] After downsample ({self.ply_voxel_size}m): "
                     f"{len(all_pts):,} pts "
                     f"({100 * len(all_pts) / max(1, total_raw):.1f}%)")

        if len(all_pts) == 0:
            self.log("[PLY] No points after filtering.")
            return None

        colors = self._height_to_rgb(all_pts[:, 2])

        out_dir = os.path.dirname(os.path.abspath(filepath))
        os.makedirs(out_dir, exist_ok=True)

        self.log(f"[PLY] Writing {len(all_pts):,} points to: {filepath}")
        with open(filepath, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"comment Generated by AutonomousExplorer full-coverage scan\n")
            f.write(f"comment Voxel size: {self.ply_voxel_size}m\n")
            f.write(f"comment Total scans: {len(self.all_lidar_points)}\n")
            f.write(f"comment Raw points: {total_raw}\n")
            f.write(f"element vertex {len(all_pts)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            for i in range(len(all_pts)):
                f.write(f"{all_pts[i, 0]:.4f} {all_pts[i, 1]:.4f} "
                        f"{all_pts[i, 2]:.4f} "
                        f"{colors[i, 0]} {colors[i, 1]} {colors[i, 2]}\n")

        file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
        self.log(f"[PLY] Saved! {file_size_mb:.2f} MB")
        return filepath

    # ================================================================
    #  MAIN EXPLORATION LOOP
    # ================================================================

    def explore_loop(self):
        """
        Full-coverage exploration in three phases:

        Phase 1 - Initial 360° scan from start to detect bounds/walls.
        Phase 2 - Systematic lawnmower flight with 360° scans at each
                  waypoint for dense, complete point cloud.
        Phase 3 - BFS cleanup for any free cells missed by the lawnmower
                  (areas behind obstacles, etc.).
        """

        # ---- Phase 1: Initial full scan ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 1: Initial 360 scan")
        self.log("=" * 60)
        self.scan_full()

        free_count = int(np.sum(self.grid == EXPLORED))
        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        self.log(f"[PHASE 1] After initial scan: {visited_count}/{free_count} "
                 f"cells visited, {int(np.sum(self.grid == WALL))} wall cells")
        self.print_grid(compact=True)

        # ---- Phase 2: Lawnmower coverage ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 2: Lawnmower coverage flight")
        self.log("=" * 60)

        waypoints = self.generate_coverage_waypoints()
        total_wp = len(waypoints)
        self.log(f"[PHASE 2] Generated {total_wp} coverage waypoints "
                 f"(spacing={self.coverage_spacing}m)")

        reached = 0
        skipped = 0

        nav_failures = 0
        max_consec_fail = 10  # re-scan after N consecutive failures

        for i, (wx, wy) in enumerate(waypoints):
            if not self.is_running_fn():
                self.log("[PHASE 2] Stopped by user.")
                break

            gx, gy = self.world_to_grid(wx, wy)

            # Skip already-visited cells
            if self.visited[gx][gy]:
                skipped += 1
                continue

            # Skip wall or unknown cells (drone can't fly into unknowns safely)
            if self.grid[gx][gy] != EXPLORED:
                skipped += 1
                continue

            # Navigate to waypoint
            success = self.navigate_to_world(wx, wy)

            if success:
                reached += 1
                nav_failures = 0
                self.scan_full()
            else:
                skipped += 1
                nav_failures += 1
                # After many consecutive failures, do a 360° rescan to update map
                if nav_failures >= max_consec_fail:
                    self.log(f"[PHASE 2] {nav_failures} consecutive nav failures, rescanning...")
                    self.scan_full()
                    nav_failures = 0

            # Progress report
            visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
            free_count = int(np.sum(self.grid == EXPLORED))
            pct = 100 * visited_count / max(1, free_count)

            if reached > 0 and reached % 3 == 0:
                self.log(f"[PHASE 2] WP {i + 1}/{total_wp} | "
                         f"Reached: {reached} | Skipped: {skipped} | "
                         f"Coverage: {visited_count}/{free_count} ({pct:.0f}%)")

        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        free_count = int(np.sum(self.grid == EXPLORED))
        pct = 100 * visited_count / max(1, free_count)
        self.log(f"[PHASE 2] Complete. Reached: {reached}, Skipped: {skipped}")
        self.log(f"[PHASE 2] Coverage: {visited_count}/{free_count} ({pct:.0f}%)")
        self.print_grid(compact=True)

        # ---- Phase 3: BFS cleanup ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 3: BFS cleanup for missed areas")
        self.log("=" * 60)

        cleanup_steps = 0
        cleanup_fails = 0
        max_cleanup = self.grid_dim * self.grid_dim
        max_cleanup_fails = 15  # give up after N consecutive BFS failures

        while cleanup_steps < max_cleanup and cleanup_fails < max_cleanup_fails:
            if not self.is_running_fn():
                break

            path = self.find_path_to_nearest_unvisited()
            if path is None:
                break

            cleanup_steps += 1

            # Navigate along BFS path
            target_gx, target_gy = path[-1]
            target_wx, target_wy = self.grid_to_world(target_gx, target_gy)
            success = self.navigate_to_world(target_wx, target_wy)

            if success:
                cleanup_fails = 0
                self.scan_full()
            else:
                cleanup_fails += 1
                # Mark unreachable target to avoid retrying
                if self.grid[target_gx][target_gy] == EXPLORED:
                    self.visited[target_gx][target_gy] = True

            # Progress
            if cleanup_steps % 3 == 0:
                visited_count = int(np.sum(
                    self.visited & (self.grid == EXPLORED)))
                free_count = int(np.sum(self.grid == EXPLORED))
                self.log(f"[PHASE 3] Step {cleanup_steps}: "
                         f"{visited_count}/{free_count} "
                         f"({100 * visited_count / max(1, free_count):.0f}%)")

        if cleanup_steps > 0:
            self.log(f"[PHASE 3] Cleanup: {cleanup_steps} additional positions")
        else:
            self.log("[PHASE 3] No cleanup needed - full coverage!")

        # ---- Final stats ----
        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        free_count = int(np.sum(self.grid == EXPLORED))
        wall_count = int(np.sum(self.grid == WALL))
        coverage = 100 * visited_count / max(1, free_count)

        self.log("")
        self.log("=" * 60)
        self.log("  EXPLORATION COMPLETE")
        self.log(f"  Free cells:      {free_count}")
        self.log(f"  Wall cells:      {wall_count}")
        self.log(f"  Visited:         {visited_count}/{free_count} "
                 f"({coverage:.0f}% coverage)")
        self.log(f"  Trajectory pts:  {len(self.trajectory_points)}")
        self.log(f"  Lidar scans:     {len(self.all_lidar_points)}")
        self.log("=" * 60)

        self.print_grid(compact=True)

    # ================================================================
    #  RUN (FULL LIFECYCLE)
    # ================================================================

    def run(self, log_callback=None, running_check=None):
        """Full exploration lifecycle."""
        if log_callback:
            self.log = log_callback
        if running_check:
            self.is_running_fn = running_check

        try:
            if not self.external_client:
                self.client.enableApiControl(True)
                self.client.armDisarm(True)

            self.takeoff()

            if self.auto_room_size:
                self._auto_configure_room_from_lidar()

            self.explore_loop()

            if self.ply_output_path:
                self.save_point_cloud_ply()

            self.return_home_smooth()

        except Exception as e:
            self.log(f"[ERROR] Exploration failed: {e}")
            import traceback
            traceback.print_exc()
            try:
                self.log("[EMERGENCY] Attempting emergency landing...")
                self.client.landAsync().join()
            except Exception:
                pass
        finally:
            if not self.external_client:
                try:
                    self.client.armDisarm(False)
                    self.client.enableApiControl(False)
                except Exception:
                    pass
            self.log("[INFO] Exploration session ended.")


# ================================================================
#  STANDALONE EXECUTION
# ================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Autonomous Full-Coverage Drone Room Explorer"
    )
    parser.add_argument("--mock", action="store_true",
                        help="Use mock AirSim environment")
    parser.add_argument("--allow-mock-fallback", action="store_true",
                        help="Allow mock fallback if AirSim unavailable")
    parser.add_argument("--room-size", type=float, default=20.0,
                        help="Room size in meters (default: 20)")
    parser.add_argument("--cell-size", type=float, default=2.0,
                        help="Grid cell size in meters (default: 2)")
    parser.add_argument("--altitude", type=float, default=2.0,
                        help="Flight altitude in meters (default: 2)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="Flight speed m/s (default: 2)")
    parser.add_argument("--safe-distance", type=float, default=0.8,
                        help="Min obstacle distance in meters (default: 0.8)")
    parser.add_argument("--auto-room-size", action="store_true",
                        help="Auto-detect room size from first lidar scan")
    parser.add_argument("--auto-room-padding", type=float, default=1.0,
                        help="Extra map padding (default: 1)")
    parser.add_argument("--scan-rotations", type=int, default=12,
                        help="Rotation steps for 360 scan (default: 12)")
    parser.add_argument("--visit-radius", type=float, default=5.0,
                        help="Radius to mark cells visited (default: 5)")
    parser.add_argument("--coverage-spacing", type=float, default=5.0,
                        help="Lawnmower row spacing in meters (default: 5)")
    parser.add_argument("--ply-output", type=str, default="point_cloud.ply",
                        help="Output PLY file (default: point_cloud.ply)")
    parser.add_argument("--ply-voxel", type=float, default=0.05,
                        help="Voxel downsample size (default: 0.05)")
    parser.add_argument("--no-ply", action="store_true",
                        help="Disable PLY export")
    args = parser.parse_args()

    explorer = AutonomousExplorer(
        use_mock=args.mock,
        room_size=args.room_size,
        cell_size=args.cell_size,
        altitude=args.altitude,
        speed=args.speed,
        safe_distance=args.safe_distance,
        require_sim=(not args.mock and not args.allow_mock_fallback),
        auto_room_size=args.auto_room_size,
        auto_room_padding=args.auto_room_padding,
        scan_rotations=args.scan_rotations,
        visit_radius=args.visit_radius,
        coverage_spacing=args.coverage_spacing,
    )

    if not args.no_ply:
        explorer.ply_output_path = args.ply_output
        explorer.ply_voxel_size = args.ply_voxel

    explorer.run()
