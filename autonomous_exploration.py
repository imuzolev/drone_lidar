"""
Autonomous Full-Coverage Explorer for AirSim Drones.

Uses frontier-based exploration to fully discover and scan any building,
including warehouses with shelving aisles, corridors, L-shaped rooms, and
arbitrary geometry.  No prior knowledge of building layout is required.

The drone iteratively flies to the boundary between known and unknown
space (frontiers), performs 360° lidar scans, builds a dense point cloud,
avoids obstacles, and returns home after full coverage.

Exploration phases:
  Phase 1 - Initial 360° scan to map immediate surroundings.
  Phase 2 - Frontier exploration: discover entire reachable building by
            flying to frontier cells (known/unknown boundary).
  Phase 3 - Coverage pass: visit remaining mapped-but-unvisited areas
            for dense point cloud coverage.
  Phase 4 - Return to start with smooth descent and landing.

Usage:
    python autonomous_exploration.py                      # Real AirSim
    python autonomous_exploration.py --mock               # Mock mode
    python autonomous_exploration.py --auto-room-size     # Auto-detect map
    python autonomous_exploration.py --allow-mock-fallback
    python autonomous_exploration.py --room-size 300      # Large warehouse
"""

import time
import os
import numpy as np
import math
from collections import deque
import sys
import argparse
import threading

try:
    from scipy.spatial import KDTree
except ImportError:
    KDTree = None

# Try to import airsim
try:
    import airsim
except ImportError:
    airsim = None

try:
    import mock_airsim
except ImportError:
    try:
        from src import mock_airsim
    except ImportError:
        mock_airsim = None

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
                 cell_size=2.0, altitude=3.0, speed=1.5, safe_distance=2.0,
                 require_sim=True, auto_room_size=False, auto_room_padding=1.0,
                 scan_rotations=12, visit_radius=2.5, coverage_spacing=3.0,
                 lidar_min_range=0.5, lidar_max_range=40.0,
                 ply_height_min=None, ply_height_max=None,
                 sor_neighbors=20, sor_std_ratio=2.0,
                 ror_radius=0.5, ror_min_neighbors=5,
                 visualize_in_sim=True, viz_update_interval=3):
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
            lidar_min_range:  Ignore lidar points closer than this (m).
            lidar_max_range:  Ignore lidar points farther than this (m).
            ply_height_min:   Min Z to keep in PLY (None = auto from altitude).
            ply_height_max:   Max Z to keep in PLY (None = auto from altitude).
            sor_neighbors:    Neighbours for Statistical Outlier Removal.
            sor_std_ratio:    Std deviation multiplier for SOR threshold.
            ror_radius:       Radius for Radius Outlier Removal (m).
            ror_min_neighbors: Min neighbours within ror_radius to keep point.
            visualize_in_sim: Draw debug overlay in UE viewport (grid, paths).
            viz_update_interval: Update grid overlay every N waypoints.
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

        # Grid parameters (rectangular support)
        self.room_size = room_size
        self.room_size_x = room_size
        self.room_size_y = room_size
        self.cell_size = cell_size
        self.grid_dim = 0
        self.grid_dim_x = 0
        self.grid_dim_y = 0
        self.grid = np.zeros((1, 1), dtype=int)
        self.visited = np.zeros((1, 1), dtype=bool)
        self.world_min_x = 0.0
        self.world_min_y = 0.0
        self._configure_grid_rect(room_size, room_size, center_x=0.0, center_y=0.0)

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

        # Noise filtering parameters (per-scan)
        self.lidar_min_range = max(0.1, float(lidar_min_range))
        self.lidar_max_range = max(1.0, float(lidar_max_range))

        # PLY post-processing parameters
        self.ply_height_min = ply_height_min
        self.ply_height_max = ply_height_max
        self.sor_neighbors = max(1, int(sor_neighbors))
        self.sor_std_ratio = max(0.1, float(sor_std_ratio))
        self.ror_radius = max(0.01, float(ror_radius))
        self.ror_min_neighbors = max(1, int(ror_min_neighbors))

        # State
        self.start_position = None
        self.trajectory_points = []

        # Point cloud accumulation for PLY export
        self.all_lidar_points = []
        self.ply_output_path = None
        self.ply_voxel_size = 0.05

        # In-sim visualization
        self.visualize_in_sim = bool(visualize_in_sim)
        self.viz_update_interval = max(1, int(viz_update_interval))

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
    #  KEEPALIVE & HOVER HOLD
    # ================================================================

    def _keepalive_hover(self):
        """Issue a hoverAsync command to prevent AirSim's 'hover mode for
        safety' timeout during long computation pauses (BFS, grid updates).
        Safe to call at any time — simply tells the drone to hold position."""
        try:
            self.client.hoverAsync()
        except Exception:
            pass

    # ================================================================
    #  ASYNC JOIN WITH TIMEOUT
    # ================================================================

    def _join_with_timeout(self, task, timeout=30.0, task_name="task"):
        """Join an async AirSim task with a timeout.

        AirSim's .join() can block forever when the drone enters hover-
        for-safety mode or cannot reach the target.  This wrapper runs
        .join() on a daemon thread and waits up to *timeout* seconds.
        If the task does not finish in time it is cancelled and the
        drone is stabilised in hover mode.

        Returns True if the task completed normally, False on timeout.
        """
        completed = threading.Event()

        def _wait():
            try:
                task.join()
            except Exception:
                pass
            finally:
                completed.set()

        t = threading.Thread(target=_wait, daemon=True)
        t.start()

        if completed.wait(timeout=timeout):
            return True

        # Timeout reached
        self.log(f"  [TIMEOUT] '{task_name}' exceeded {timeout:.0f}s — cancelling.")
        try:
            self.client.cancelLastTask()
        except Exception:
            pass
        try:
            self.client.hoverAsync()
        except Exception:
            pass
        time.sleep(0.3)
        return False

    def _move_timeout(self, distance, speed):
        """Calculate a reasonable timeout for a move command (seconds)."""
        if speed <= 0:
            speed = 0.5
        return max(10.0, (distance / speed) * 3.0 + 5.0)

    def _wait_for_stable(self, timeout=2.0, vel_threshold=0.15):
        """Wait until drone velocity drops below threshold (stabilized hover).

        Prevents lidar collection while the drone is still oscillating
        after a move or rotation command, which would cause point cloud
        smearing and distortions.

        Returns True if stabilized within timeout, False otherwise.
        """
        if self.use_mock:
            time.sleep(0.1)
            return True

        start = time.time()
        while time.time() - start < timeout:
            try:
                state = self.client.simGetGroundTruthKinematics()
                vx = state.linear_velocity.x_val
                vy = state.linear_velocity.y_val
                vz = state.linear_velocity.z_val
                speed = math.sqrt(vx * vx + vy * vy + vz * vz)

                av = state.angular_velocity
                ang_speed = math.sqrt(av.x_val**2 + av.y_val**2 + av.z_val**2)

                if speed < vel_threshold and ang_speed < 0.1:
                    return True
            except Exception:
                break
            time.sleep(0.05)
        return False

    # ================================================================
    #  GRID CONFIGURATION
    # ================================================================

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
        # Collect points from a full rotation for better coverage
        all_pts = []
        angle_step = 360.0 / max(4, self.scan_rotations)
        for i in range(max(4, self.scan_rotations)):
            yaw = i * angle_step
            try:
                task = self.client.rotateToYawAsync(yaw, 10.0)
                self._join_with_timeout(task, timeout=8.0, task_name="auto_room_rotate")
            except Exception:
                pass
            self._wait_for_stable(timeout=1.0, vel_threshold=0.15)
            time.sleep(0.1)
            pts = self.get_lidar_points_world()
            if len(pts) > 0:
                all_pts.append(pts)

        if not all_pts:
            self.log("[AUTO ROOM] Lidar returned no points. Keeping default room size.")
            return

        world_pts = np.concatenate(all_pts, axis=0)

        # IQR-based outlier rejection per axis — keeps sparse far-wall
        # points but removes truly errant hits (through openings, etc.)
        for axis in range(2):  # X=0, Y=1
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

        # Actual min/max of inlier points — no percentile center-shift
        x_lo = float(world_pts[:, 0].min())
        x_hi = float(world_pts[:, 0].max())
        y_lo = float(world_pts[:, 1].min())
        y_hi = float(world_pts[:, 1].max())
        span_x = x_hi - x_lo
        span_y = y_hi - y_lo

        self.log(f"[AUTO ROOM] Lidar bounds X=[{x_lo:.1f}, {x_hi:.1f}] "
                 f"({span_x:.1f}m)  Y=[{y_lo:.1f}, {y_hi:.1f}] ({span_y:.1f}m)")

        # Rectangular room support: track X / Y sizes separately
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
        """Recalibrate grid center using discovered WALL cells.
        Called after Phase 2 perimeter pass when we have good wall data.
        Shifts the grid so that walls are centered without losing data."""
        wall_positions = np.argwhere(self.grid == WALL)
        if len(wall_positions) < 4:
            self.log("[RECALIBRATE] Not enough wall data to recalibrate.")
            return

        # Convert wall grid positions to world coordinates
        wall_world_x = self.world_min_x + wall_positions[:, 0] * self.cell_size + self.cell_size / 2
        wall_world_y = self.world_min_y + wall_positions[:, 1] * self.cell_size + self.cell_size / 2

        # Actual wall bounds in world coords
        wx_lo = float(wall_world_x.min())
        wx_hi = float(wall_world_x.max())
        wy_lo = float(wall_world_y.min())
        wy_hi = float(wall_world_y.max())

        actual_center_x = (wx_lo + wx_hi) / 2.0
        actual_center_y = (wy_lo + wy_hi) / 2.0

        # Current grid center
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

        # Rebuild grid with corrected center, keeping size
        # Add extra padding to ensure walls fit
        wall_span_x = wx_hi - wx_lo
        wall_span_y = wy_hi - wy_lo
        pad = self.auto_room_padding
        new_size_x = max(self.room_size_x, wall_span_x + 2.0 * pad)
        new_size_y = max(self.room_size_y, wall_span_y + 2.0 * pad)

        # Save old grid data
        old_grid = self.grid.copy()
        old_visited = self.visited.copy()
        old_min_x = self.world_min_x
        old_min_y = self.world_min_y
        old_dim_x = self.grid_dim_x
        old_dim_y = self.grid_dim_y

        # Rebuild with corrected center
        self._configure_grid_rect(new_size_x, new_size_y,
                                  center_x=actual_center_x,
                                  center_y=actual_center_y)

        # Copy old grid data into new grid
        for ogx in range(old_dim_x):
            for ogy in range(old_dim_y):
                if old_grid[ogx][ogy] != UNEXPLORED or old_visited[ogx][ogy]:
                    # Convert old grid cell to world, then to new grid
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

    # ================================================================
    #  COORDINATE CONVERSION
    # ================================================================

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

    # ================================================================
    #  DRONE STATE
    # ================================================================

    def get_position(self):
        """Get current position as (x, y, z) tuple."""
        # Use Ground Truth to eliminate drift and ensure perfect map alignment
        if not self.use_mock and self.client:
            try:
                state = self.client.simGetGroundTruthKinematics()
                p = state.position
                return p.x_val, p.y_val, p.z_val
            except Exception:
                pass
        
        state = self.client.getMultirotorState()
        p = state.kinematics_estimated.position
        return p.x_val, p.y_val, p.z_val

    def get_yaw(self):
        """Get current yaw angle in radians."""
        # Use Ground Truth to eliminate drift
        if not self.use_mock and self.client:
            try:
                state = self.client.simGetGroundTruthKinematics()
                o = state.orientation
                siny = 2.0 * (o.w_val * o.z_val + o.x_val * o.y_val)
                cosy = 1.0 - 2.0 * (o.y_val ** 2 + o.z_val ** 2)
                return math.atan2(siny, cosy)
            except Exception:
                pass

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
    #  IN-SIM DEBUG VISUALIZATION
    # ================================================================

    def visualize_grid_in_sim(self, duration=20.0):
        """Draw coloured grid overlay in UE viewport.

        Cell colours:
          - Blue (transparent)  : UNEXPLORED
          - Yellow              : EXPLORED but not yet visited by drone
          - Green               : Visited (drone flew here & scanned)
          - Red                 : WALL / obstacle
        """
        if not self.visualize_in_sim or self.use_mock or airsim is None:
            return

        unexplored_pts = []
        explored_pts = []
        visited_pts = []
        wall_pts = []

        viz_z = self.target_altitude + 0.3

        for gx in range(self.grid_dim_x):
            for gy in range(self.grid_dim_y):
                wx, wy = self.grid_to_world(gx, gy)
                pt = airsim.Vector3r(wx, wy, viz_z)

                if self.grid[gx][gy] == WALL:
                    wall_pts.append(pt)
                elif self.visited[gx][gy]:
                    visited_pts.append(pt)
                elif self.grid[gx][gy] == EXPLORED:
                    explored_pts.append(pt)
                else:
                    unexplored_pts.append(pt)

        point_size = max(8.0, self.cell_size * 3.0)

        try:
            if wall_pts:
                self.client.simPlotPoints(
                    wall_pts,
                    color_rgba=[1.0, 0.0, 0.0, 0.85],
                    size=point_size, duration=duration,
                    is_persistent=False
                )
            if visited_pts:
                self.client.simPlotPoints(
                    visited_pts,
                    color_rgba=[0.0, 1.0, 0.0, 0.55],
                    size=point_size, duration=duration,
                    is_persistent=False
                )
            if explored_pts:
                self.client.simPlotPoints(
                    explored_pts,
                    color_rgba=[1.0, 1.0, 0.0, 0.50],
                    size=point_size, duration=duration,
                    is_persistent=False
                )
            if unexplored_pts:
                self.client.simPlotPoints(
                    unexplored_pts,
                    color_rgba=[0.2, 0.3, 1.0, 0.30],
                    size=point_size, duration=duration,
                    is_persistent=False
                )
        except Exception as e:
            self.log(f"  [VIZ] Grid overlay error: {e}")

    def visualize_planned_path(self, waypoints, color_rgba,
                               thickness=5.0, duration=120.0,
                               label=None):
        """Draw a planned waypoint path and its markers in UE viewport.

        Args:
            waypoints: list of (wx, wy) world-coordinate tuples.
            color_rgba: [r, g, b, a] colour for the line and markers.
            thickness: line thickness in pixels.
            duration: how long the drawing persists (seconds).
            label: optional text label at the first waypoint.
        """
        if not self.visualize_in_sim or self.use_mock or airsim is None:
            return
        if len(waypoints) < 2:
            return

        pts = [
            airsim.Vector3r(wx, wy, self.target_altitude)
            for wx, wy in waypoints
        ]

        try:
            self.client.simPlotLineStrip(
                pts, color_rgba=color_rgba,
                thickness=thickness, duration=duration,
                is_persistent=False
            )
            self.client.simPlotPoints(
                pts, color_rgba=color_rgba,
                size=12.0, duration=duration,
                is_persistent=False
            )
            if label and len(pts) > 0:
                try:
                    self.client.simPlotStrings(
                        [label], [pts[0]],
                        scale=1.5,
                        color_rgba=color_rgba,
                        duration=duration
                    )
                except Exception:
                    pass
        except Exception as e:
            self.log(f"  [VIZ] Planned path error: {e}")

    def visualize_current_target(self, target_x, target_y,
                                  bfs_path=None):
        """Highlight current navigation target and optional BFS route.

        Draws:
          - Large white point at current target.
          - Orange line along BFS grid path (if provided).
        """
        if not self.visualize_in_sim or self.use_mock or airsim is None:
            return

        try:
            target_pt = airsim.Vector3r(target_x, target_y,
                                        self.target_altitude)
            self.client.simPlotPoints(
                [target_pt],
                color_rgba=[1.0, 1.0, 1.0, 1.0],
                size=25.0, duration=10.0,
                is_persistent=False
            )

            if bfs_path and len(bfs_path) >= 2:
                path_pts = [
                    airsim.Vector3r(
                        *self.grid_to_world(gx, gy),
                        self.target_altitude
                    )
                    for gx, gy in bfs_path
                ]
                self.client.simPlotLineStrip(
                    path_pts,
                    color_rgba=[1.0, 0.6, 0.0, 0.9],
                    thickness=6.0, duration=10.0,
                    is_persistent=False
                )
        except Exception as e:
            self.log(f"  [VIZ] Target marker error: {e}")

    def visualize_phase_label(self, phase_name):
        """Draw a large phase label near the drone position."""
        if not self.visualize_in_sim or self.use_mock or airsim is None:
            return

        try:
            x, y, z = self.get_position()
            pos = airsim.Vector3r(x, y, self.target_altitude - 2.0)
            self.client.simPlotStrings(
                [phase_name], [pos],
                scale=2.5,
                color_rgba=[1.0, 1.0, 1.0, 1.0],
                duration=15.0
            )
        except Exception:
            pass

    # ================================================================
    #  LIDAR PROCESSING
    # ================================================================

    def _get_orientation_quaternion(self):
        """Get current orientation as (w, x, y, z) quaternion using ground truth."""
        if not self.use_mock and self.client:
            try:
                state = self.client.simGetGroundTruthKinematics()
                o = state.orientation
                return o.w_val, o.x_val, o.y_val, o.z_val
            except Exception:
                pass
        state = self.client.getMultirotorState()
        o = state.kinematics_estimated.orientation
        return o.w_val, o.x_val, o.y_val, o.z_val

    def get_lidar_points_world(self):
        """Get lidar point cloud converted to world coordinates.
        Uses full quaternion rotation for accurate body-to-world transform.
        Applies per-scan noise filtering: NaN removal, range gating.
        Returns numpy array (N, 3)."""
        try:
            lidar_data = self.client.getLidarData()
        except Exception:
            return np.array([]).reshape(0, 3)

        if len(lidar_data.point_cloud) < 3:
            return np.array([]).reshape(0, 3)

        points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

        # --- Per-scan noise filter 1: Remove NaN/Inf ---
        valid_mask = np.all(np.isfinite(points), axis=1)
        points = points[valid_mask]
        if len(points) == 0:
            return np.array([]).reshape(0, 3)

        # --- Per-scan noise filter 2: Range gating (body frame) ---
        distances = np.linalg.norm(points, axis=1)
        range_mask = (distances >= self.lidar_min_range) & (distances <= self.lidar_max_range)
        points = points[range_mask]
        if len(points) == 0:
            return np.array([]).reshape(0, 3)

        # Full quaternion rotation for accurate body-to-world transform
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

        cone_half_angle = math.radians(20.0)
        cone_mask = angle_diff < cone_half_angle
        cone_points = points[cone_mask]

        if len(cone_points) == 0:
            return float('inf')

        distances = np.linalg.norm(cone_points[:, :2], axis=1)
        return float(np.min(distances))

    # ================================================================
    #  GRID MAPPING
    # ================================================================

    def update_grid_from_lidar(self, collect_for_ply=True):
        """Update occupancy grid from lidar scan.
        Marks WALL at lidar endpoints, EXPLORED along rays (free space).
        Only accumulates points for PLY export when collect_for_ply=True.
        During transit, collect_for_ply should be False to avoid
        capturing noisy points while the drone is in motion."""
        x, y, z = self.get_position()
        drone_gx, drone_gy = self.world_to_grid(x, y)

        # Mark cells near drone as free
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

        # Accumulate for PLY with height-band filtering (only when stabilized)
        if len(world_pts) > 0 and self.ply_output_path is not None and collect_for_ply:
            ply_pts = world_pts.copy()

            # Height-band filter: remove floor/ceiling noise
            z_drone = z  # current drone altitude (NED negative)
            h_min = self.ply_height_min
            h_max = self.ply_height_max
            if h_min is None:
                h_min = z_drone - 8.0  # ~8m below drone
            if h_max is None:
                h_max = z_drone + 3.0  # ~3m above drone

            height_mask = (ply_pts[:, 2] >= h_min) & (ply_pts[:, 2] <= h_max)
            ply_pts = ply_pts[height_mask]

            if len(ply_pts) > 0:
                self.all_lidar_points.append(ply_pts)

        # Height filter: only mark as WALL if lidar hit is near flight altitude.
        # Floor/ceiling hits should not block 2D navigation grid.
        z_wall_tolerance = 1.0  # meters above/below drone altitude

        for pt in world_pts:
            wx, wy, wz = pt[0], pt[1], pt[2]
            wgx, wgy = self.world_to_grid(wx, wy)
            if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
                # Only mark as wall if the hit point is near flight altitude
                # AND the cell hasn't been physically visited (prevents
                # false walls from fragmenting known-safe space)
                if abs(wz - z) <= z_wall_tolerance:
                    if not self.visited[wgx][wgy]:
                        self.grid[wgx][wgy] = WALL
            # Always ray-trace to mark free space between drone and hit point
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

    # ================================================================
    #  360° SCANNING
    # ================================================================

    def scan_full(self):
        """Rotate 360° in steps, collecting lidar at each angle.
        Marks nearby cells as visited after scanning.
        Waits for full stabilization before each lidar capture
        to prevent point cloud smearing."""
        angle_step = 360.0 / self.scan_rotations
        for i in range(self.scan_rotations):
            if not self.is_running_fn():
                break
            yaw = i * angle_step
            try:
                task = self.client.rotateToYawAsync(yaw, 10.0)
                self._join_with_timeout(task, timeout=8.0, task_name="scan_rotate")
            except Exception:
                pass
            self._wait_for_stable(timeout=1.5, vel_threshold=0.1)
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
                if 0 <= ngx < self.grid_dim_x and 0 <= ngy < self.grid_dim_y:
                    cx, cy = self.grid_to_world(ngx, ngy)
                    dist = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
                    if dist <= self.visit_radius:
                        self.visited[ngx][ngy] = True

    # ================================================================
    #  ADAPTIVE SCANNING
    # ================================================================

    def _unexplored_ratio_at(self, gx, gy):
        """Ratio of UNEXPLORED cells within visit_radius of a grid position.
        Uses numpy slicing for speed on large grids."""
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
        """Adaptive 360° scan: full rotation in unmapped areas, reduced in
        well-mapped areas.  Always collects lidar at each orientation so PLY
        point-cloud density is preserved."""
        x, y, _ = self.get_position()
        gx, gy = self.world_to_grid(x, y)
        ratio = self._unexplored_ratio_at(gx, gy)

        if ratio > 0.15:
            rotations = self.scan_rotations          # full (default 12)
        elif ratio > 0.05:
            rotations = max(6, self.scan_rotations * 2 // 3)  # ~8
        else:
            rotations = max(4, self.scan_rotations // 2)       # ~6

        angle_step = 360.0 / rotations
        for i in range(rotations):
            if not self.is_running_fn():
                break
            yaw = i * angle_step
            try:
                task = self.client.rotateToYawAsync(yaw, 10.0)
                self._join_with_timeout(task, timeout=8.0, task_name="adaptive_rotate")
            except Exception:
                pass
            self._wait_for_stable(timeout=1.5, vel_threshold=0.1)
            time.sleep(0.15)
            self.update_grid_from_lidar()

        self.mark_cells_visited()
        self.record_trajectory()

    # ================================================================
    #  COVERAGE PATH GENERATION (LAWNMOWER)
    # ================================================================

    def is_location_reachable(self, gx, gy):
        """Lightweight reachability check for waypoint planning.
        Only verifies the target cell itself is not a wall.
        Runtime obstacle avoidance in navigate_to_cell() handles
        proximity to walls — no need to over-filter at planning stage."""
        if not (0 <= gx < self.grid_dim_x and 0 <= gy < self.grid_dim_y):
            return False
        return self.grid[gx][gy] != WALL

    def is_location_safe(self, gx, gy):
        """Check if grid cell is safe (far enough from known walls).
        Returns True if distance to nearest known WALL >= safe_distance.
        """
        if not (0 <= gx < self.grid_dim_x and 0 <= gy < self.grid_dim_y):
            return False

        if self.grid[gx][gy] == WALL:
            return False

        # Check radius in grid cells
        safe_radius_cells = int(math.ceil(self.safe_distance / self.cell_size))

        min_gx = max(0, gx - safe_radius_cells)
        max_gx = min(self.grid_dim_x - 1, gx + safe_radius_cells)
        min_gy = max(0, gy - safe_radius_cells)
        max_gy = min(self.grid_dim_y - 1, gy + safe_radius_cells)

        # Quick check: are there any walls in the bounding box?
        subgrid = self.grid[min_gx:max_gx+1, min_gy:max_gy+1]
        if not np.any(subgrid == WALL):
            return True

        # Detailed distance check
        tx, ty = self.grid_to_world(gx, gy)
        
        # Get indices of all walls in subgrid
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
        """Generate waypoints along the room perimeter for edge/corner coverage.
        Starts from the nearest corner and traverses all 4 edges."""
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

        # Remove near-duplicate points
        filtered = [waypoints[0]]
        for wp in waypoints[1:]:
            if math.sqrt((wp[0] - filtered[-1][0])**2 +
                         (wp[1] - filtered[-1][1])**2) > spacing * 0.4:
                filtered.append(wp)

        # Filter by reachability
        valid = []
        for wx, wy in filtered:
            gx, gy = self.world_to_grid(wx, wy)
            if self.is_location_reachable(gx, gy):
                valid.append((wx, wy))

        return valid

    def _generate_interior_waypoints(self):
        """Generate interior boustrophedon (lawnmower) waypoints.
        Slight offset from perimeter for overlap, uses relaxed safety check."""
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

        # Filter by relaxed reachability
        valid = []
        for wx, wy in raw_waypoints:
            gx, gy = self.world_to_grid(wx, wy)
            if self.is_location_reachable(gx, gy):
                valid.append((wx, wy))

        return valid

    def generate_coverage_waypoints(self):
        """Generate full coverage waypoints: perimeter + interior lawnmower.
        Filters out waypoints near known walls."""
        return self._generate_perimeter_waypoints() + self._generate_interior_waypoints()

    def _generate_fill_waypoints(self, spacing=None):
        """Generate waypoints targeting only unvisited reachable cells.
        Produces a lawnmower pattern with the given spacing but filters
        out cells that are already visited, resulting in focused coverage."""
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

        # Keep only unvisited reachable waypoints
        valid = []
        for wx, wy in raw_waypoints:
            gx, gy = self.world_to_grid(wx, wy)
            if (self.is_location_reachable(gx, gy) and
                    not self.visited[gx][gy]):
                valid.append((wx, wy))

        return valid

    # ================================================================
    #  OBSTACLE CIRCUMNAVIGATION
    # ================================================================

    def _detect_internal_obstacles(self):
        """Detect clusters of WALL cells that are internal obstacles
        (not part of the room perimeter walls).

        Uses flood-fill (BFS) to group connected WALL cells into clusters.
        Filters out clusters that touch the grid boundary (room walls).

        Returns:
            List of clusters, each cluster is a list of (gx, gy) tuples.
        """
        wall_mask = (self.grid == WALL)
        visited_cells = np.zeros_like(wall_mask, dtype=bool)
        clusters = []
        border = 2  # cells from edge considered "perimeter"

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
        """Generate waypoints around each internal obstacle so the drone
        can scan all sides.

        For each obstacle cluster:
          1. Find centroid and radius.
          2. Place waypoints in a circle at (radius + safe_distance + margin).
          3. Use 8 evenly spaced angles (every 45°) for full coverage.
          4. Filter out unreachable / wall waypoints.

        Returns:
            List of (obstacle_label, waypoints) tuples.
        """
        clusters = self._detect_internal_obstacles()
        if not clusters:
            return []

        self.log(f"[OBSTACLES] Detected {len(clusters)} internal obstacle(s)")

        all_obstacle_wps = []
        orbit_angles = 8  # scan from 8 directions (every 45°)
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

    # ================================================================
    #  PATH PLANNING
    # ================================================================

    def _is_passable(self, gx, gy):
        """Check if a grid cell can be traversed.
        A cell is passable if it's EXPLORED (known free space) or if
        the drone has physically visited it (safe to fly through even
        if later re-marked as WALL by a distant scan)."""
        if self.grid[gx][gy] == EXPLORED:
            return True
        if self.visited[gx][gy]:
            return True
        return False

    def bfs_path_to(self, start_gx, start_gy, target_gx, target_gy):
        """BFS from (start) to (target) through passable cells.
        Expands through EXPLORED cells and previously visited cells.
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
            for dgx, dgy in [(1,0),(-1,0),(0,1),(0,-1),
                              (1,1),(1,-1),(-1,1),(-1,-1)]:
                nx, ny = gx + dgx, gy + dgy
                nb = (nx, ny)
                if (0 <= nx < self.grid_dim_x and 0 <= ny < self.grid_dim_y
                        and nb not in seen
                        and self._is_passable(nx, ny)):
                    # Prevent diagonal corner-cutting through walls
                    if abs(dgx) + abs(dgy) == 2:
                        if (self.grid[gx + dgx][gy] == WALL or
                                self.grid[gx][gy + dgy] == WALL):
                            continue
                    seen.add(nb)
                    parent[nb] = current
                    queue.append(nb)

        return None

    def find_path_to_nearest_unvisited(self):
        """BFS to nearest EXPLORED (free) cell that has not been visited.
        Skips UNEXPLORED cells as targets (they might be unreachable
        areas outside the building).
        Returns path as list of (gx, gy) or None."""
        x, y, _ = self.get_position()
        start = self.world_to_grid(x, y)

        queue = deque([start])
        seen = {start}
        parent = {start: None}

        while queue:
            current = queue.popleft()
            gx, gy = current

            # Target: EXPLORED cell that hasn't been visited
            # (Skip UNEXPLORED — might be outside building)
            if (self.grid[gx][gy] == EXPLORED and
                    not self.visited[gx][gy]):
                path = []
                node = current
                while node is not None:
                    path.append(node)
                    node = parent[node]
                path.reverse()
                return path

            # Expand through passable cells (EXPLORED or visited)
            for dgx, dgy in [(1, 0), (-1, 0), (0, 1), (0, -1),
                              (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = gx + dgx, gy + dgy
                nb = (nx, ny)
                if (0 <= nx < self.grid_dim_x and
                        0 <= ny < self.grid_dim_y and
                        nb not in seen and
                        self._is_passable(nx, ny)):
                    if abs(dgx) + abs(dgy) == 2:
                        if (not self._is_passable(gx + dgx, gy) or
                                not self._is_passable(gx, gy + dgy)):
                            continue
                    seen.add(nb)
                    parent[nb] = current
                    queue.append(nb)

        # Diagnostic if nothing found
        explored_unvisited = int(np.sum(
            (self.grid == EXPLORED) & ~self.visited))
        total_visited = int(np.sum(self.visited))
        self.log(f"  [BFS DEBUG] No path found. Start={start}, "
                 f"grid[start]={self.grid[start[0]][start[1]]}, "
                 f"visited[start]={self.visited[start[0]][start[1]]}, "
                 f"BFS reached {len(seen)} cells, "
                 f"explored_unvisited={explored_unvisited}, "
                 f"total_visited={total_visited}")

        return None

    def _find_nearest_frontier(self):
        """BFS to nearest frontier cell from drone position.

        A frontier cell is an EXPLORED cell that has at least one
        UNEXPLORED neighbour.  Frontiers represent the boundary between
        known free space and unknown territory — the drone should fly
        there to discover new areas.

        BFS only expands through EXPLORED cells (known free space) to
        avoid wandering through unknown/unreachable territory.

        Returns list of (gx, gy) path from drone to frontier, or None
        if no reachable frontier exists.
        """
        x, y, _ = self.get_position()
        start = self.world_to_grid(x, y)

        queue = deque([start])
        seen = {start}
        parent = {start: None}

        while queue:
            current = queue.popleft()
            gx, gy = current

            # Is this a frontier cell? (EXPLORED, adjacent to UNEXPLORED)
            if self.grid[gx][gy] == EXPLORED:
                is_frontier = False
                for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nx, ny = gx + dx, gy + dy
                    if (0 <= nx < self.grid_dim_x and
                            0 <= ny < self.grid_dim_y and
                            self.grid[nx][ny] == UNEXPLORED):
                        is_frontier = True
                        break

                if is_frontier and not self.visited[gx][gy]:
                    path = []
                    node = current
                    while node is not None:
                        path.append(node)
                        node = parent[node]
                    path.reverse()
                    return path

            # Expand BFS only through passable cells (EXPLORED or visited)
            for dgx, dgy in [(1, 0), (-1, 0), (0, 1), (0, -1),
                              (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = gx + dgx, gy + dgy
                nb = (nx, ny)
                if (0 <= nx < self.grid_dim_x and
                        0 <= ny < self.grid_dim_y and
                        nb not in seen and
                        self._is_passable(nx, ny)):
                    if abs(dgx) + abs(dgy) == 2:
                        if (not self._is_passable(gx + dgx, gy) or
                                not self._is_passable(gx, gy + dgy)):
                            continue
                    seen.add(nb)
                    parent[nb] = current
                    queue.append(nb)

        return None

    def _count_nearby_unexplored(self, gx, gy, radius=3):
        """Count UNEXPLORED cells within given grid-cell radius.
        Used for scoring frontier targets — prefer areas with more unknown."""
        count = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if (0 <= nx < self.grid_dim_x and
                        0 <= ny < self.grid_dim_y and
                        self.grid[nx][ny] == UNEXPLORED):
                    count += 1
        return count

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
            if 0 <= cx < self.grid_dim_x and 0 <= cy < self.grid_dim_y:
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

            if (abs(cx - gx0) > max(self.grid_dim_x, self.grid_dim_y) or
                    abs(cy - gy0) > max(self.grid_dim_x, self.grid_dim_y)):
                return False

        return True

    def _simplify_path(self, path):
        """Remove redundant intermediate waypoints from a BFS grid path.
        Keeps start and end; drops points where a straight line-of-sight
        exists between non-adjacent nodes.  Greatly reduces stop-and-go."""
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            farthest = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self._grid_line_clear(path[i][0], path[i][1],
                                         path[j][0], path[j][1]):
                    farthest = j
                    break
            simplified.append(path[farthest])
            i = farthest
        return simplified

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

    def navigate_to_cell(self, target_gx, target_gy, skip_yaw=False):
        """Move drone to grid cell center with adaptive speed.
        Flies as close to obstacles as possible, slowing down near walls.
        When skip_yaw=True, skips yaw rotation for smoother transit paths
        (multirotors can fly in any direction without facing it).
        Returns True if reached, False if truly blocked."""
        target_x, target_y = self.grid_to_world(target_gx, target_gy)
        x, y, z = self.get_position()

        dist_to_target = math.sqrt((target_x - x) ** 2 + (target_y - y) ** 2)

        if dist_to_target < 0.3:
            return True

        # Lidar obstacle check in direction of target
        wall_dist = self.get_distance_in_direction(target_x, target_y)
        
        # STRICT SAFETY CHECK
        # If any obstacle is closer than safe_distance in the direction of flight, STOP.
        if wall_dist < self.safe_distance:
            self.log(f"  [NAV] Wall detected at {wall_dist:.2f}m (< {self.safe_distance}m). Stopping.")
            # Mark wall roughly where it was detected
            if wall_dist < float('inf'):
                angle = math.atan2(target_y - y, target_x - x)
                wall_x = x + math.cos(angle) * wall_dist
                wall_y = y + math.sin(angle) * wall_dist
                wgx, wgy = self.world_to_grid(wall_x, wall_y)
                if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
                    self.grid[wgx][wgy] = WALL
            return False

        clearance = wall_dist - dist_to_target  # space left after reaching target

        # Hard block: we would overshoot INTO the wall
        if clearance < self.safe_distance and wall_dist < dist_to_target + 0.5:
            # Only mark the actual obstacle point as wall, not the target
            if wall_dist < float('inf'):
                angle = math.atan2(target_y - y, target_x - x)
                wall_x = x + math.cos(angle) * wall_dist
                wall_y = y + math.sin(angle) * wall_dist
                wgx, wgy = self.world_to_grid(wall_x, wall_y)
                if 0 <= wgx < self.grid_dim_x and 0 <= wgy < self.grid_dim_y:
                    self.grid[wgx][wgy] = WALL
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

        # Rotate toward target (skip during transit for smoother path)
        if not skip_yaw:
            yaw_deg = math.degrees(math.atan2(actual_target_y - y,
                                               actual_target_x - x))
            try:
                task = self.client.rotateToYawAsync(yaw_deg, 10.0)
                self._join_with_timeout(task, timeout=5.0, task_name="nav_rotate")
            except Exception:
                pass

        # Fly at adaptive speed
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

        # Post-flight collision check
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

        # Try direct flight if grid line is clear
        if self._grid_line_clear(start_gx, start_gy, target_gx, target_gy):
            wx, wy = self.grid_to_world(target_gx, target_gy)
            success = self.navigate_to_cell(target_gx, target_gy, skip_yaw=True)
            if success:
                self.update_grid_from_lidar(collect_for_ply=False)
                self.mark_cells_visited()
                return True

        # BFS pathfinding (keepalive to prevent hover-for-safety during computation)
        self._keepalive_hover()
        path = self.bfs_path_to(start_gx, start_gy, target_gx, target_gy)
        if path is None:
            return False

        # Simplify path: remove redundant intermediate waypoints
        path = self._simplify_path(path)

        # Show BFS route in viewport
        self.visualize_current_target(
            *self.grid_to_world(target_gx, target_gy),
            bfs_path=path
        )

        # Follow path smoothly: skip yaw rotation and PLY during transit
        # to avoid choppy movement and noisy point cloud data.
        # PLY collection happens only during dedicated 360° scans.
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

        # Stabilize at destination before returning (scan will follow)
        self._wait_for_stable(timeout=1.5, vel_threshold=0.15)
        self.update_grid_from_lidar(collect_for_ply=False)
        self.mark_cells_visited()
        return True

    # ================================================================
    #  FLIGHT LIFECYCLE
    # ================================================================

    def takeoff(self):
        """Take off and ascend to exploration altitude."""
        self.log("[FLIGHT] Taking off...")
        takeoff_task = self.client.takeoffAsync()
        if not self._join_with_timeout(takeoff_task, timeout=20.0,
                                       task_name="takeoff"):
            self.log("[FLIGHT] Takeoff timed out — retrying once...")
            takeoff_task = self.client.takeoffAsync()
            self._join_with_timeout(takeoff_task, timeout=15.0,
                                    task_name="takeoff_retry")

        state = self.client.getMultirotorState()
        self.start_position = state.kinematics_estimated.position
        self.log(f"[FLIGHT] Start: ({self.start_position.x_val:.1f}, "
                 f"{self.start_position.y_val:.1f})")

        self.record_trajectory()

        self.log(f"[FLIGHT] Ascending to {abs(self.target_altitude):.0f}m...")
        ascend_task = self.client.moveToPositionAsync(
            self.start_position.x_val,
            self.start_position.y_val,
            self.target_altitude,
            2.0
        )
        self._join_with_timeout(ascend_task, timeout=20.0,
                                task_name="ascend_to_altitude")
        self.record_trajectory()
        self.log("[FLIGHT] At target altitude. Ready to explore.")

    def return_home_smooth(self):
        """Smoothly return to start position using pathfinding or safe altitude."""
        if not self.start_position:
            self.log("[ERROR] Start position not recorded!")
            return

        sx = self.start_position.x_val
        sy = self.start_position.y_val
        x, y, z = self.get_position()

        self.log("[FLIGHT] === RETURNING HOME ===")

        # Try smart navigation first (avoids obstacles like the cube)
        self.log(f"[FLIGHT] Navigating to start ({sx:.1f}, {sy:.1f})...")
        success = self.navigate_to_world(sx, sy)

        if not success:
            self.log("[FLIGHT] Grid navigation failed. Executing High-Altitude Return...")
            safe_alt = min(self.target_altitude - 5.0, -10.0)

            self.log(f"[FLIGHT] Ascending to safe altitude {abs(safe_alt):.1f}m...")
            t1 = self.client.moveToPositionAsync(x, y, safe_alt, 2.0)
            self._join_with_timeout(t1, timeout=20.0, task_name="rth_ascend")

            fly_dist = math.sqrt((sx - x)**2 + (sy - y)**2)
            self.log(f"[FLIGHT] Flying over to start...")
            t2 = self.client.moveToPositionAsync(sx, sy, safe_alt, self.speed)
            self._join_with_timeout(t2, timeout=self._move_timeout(fly_dist, self.speed),
                                    task_name="rth_fly_over")

            self.log(f"[FLIGHT] Descending to approach altitude...")
            t3 = self.client.moveToPositionAsync(sx, sy, self.target_altitude, 2.0)
            self._join_with_timeout(t3, timeout=20.0, task_name="rth_descend")

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
            desc_task = self.client.moveToPositionAsync(
                sx, sy, float(-current_alt), 0.8
            )
            self._join_with_timeout(desc_task, timeout=15.0,
                                    task_name="gradual_descent")
            time.sleep(0.2)

        self.log("[FLIGHT] Descending to 0.3m...")
        final_desc = self.client.moveToPositionAsync(sx, sy, -0.3, 0.5)
        self._join_with_timeout(final_desc, timeout=15.0,
                                task_name="final_descent")
        time.sleep(0.2)
        self.record_trajectory()

        # Land with timeout
        self.log("[FLIGHT] Landing...")
        try:
            land_task = self.client.landAsync()
            self._join_with_timeout(land_task, timeout=30.0,
                                    task_name="landing")
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
        total = self.grid_dim_x * self.grid_dim_y

        self.log("")
        self.log("=" * 60)
        self.log(f"  GRID  ({self.grid_dim_x}x{self.grid_dim_y}, "
                 f"cell={self.cell_size}m, "
                 f"room={self.room_size_x:.1f}x{self.room_size_y:.1f}m)")
        self.log(f"  Free: {explored_count}  Walls: {wall_count}  "
                 f"Unexplored: {unexplored_count}")
        self.log(f"  Visited: {visited_free}/{explored_count} "
                 f"({100 * visited_free / max(1, explored_count):.0f}% coverage)")
        self.log(f"  D=drone  +=visited  .=free  ?=unknown  #=wall")
        self.log("=" * 60)

        if compact and max(self.grid_dim_x, self.grid_dim_y) > 50:
            self.log("  (grid too large for console, showing stats only)")
            self.log("=" * 60)
            return

        header = "    " + "".join(f"{i:>2}" for i in range(self.grid_dim_y))
        self.log(header)

        for gx in range(self.grid_dim_x - 1, -1, -1):
            row = f" {gx:>2} "
            for gy in range(self.grid_dim_y):
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
        """Voxel grid downsampling (snaps to grid centers to remove noise)."""
        if len(points) == 0:
            return points
        # Snap points to grid centers to merge nearby points
        quantized = np.round(points / voxel_size) * voxel_size
        # Remove duplicates
        return np.unique(quantized, axis=0)

    def _statistical_outlier_removal(self, points, nb_neighbors=20, std_ratio=2.0):
        """Remove points whose mean distance to k neighbours exceeds threshold.
        Threshold = global_mean_dist + std_ratio * global_std_dist.
        Requires scipy for KDTree; falls back to no-op if unavailable."""
        if len(points) < nb_neighbors + 1 or KDTree is None:
            if KDTree is None:
                self.log("[FILTER] scipy not available, skipping SOR")
            return points

        self.log(f"[FILTER] Statistical Outlier Removal (k={nb_neighbors}, "
                 f"std={std_ratio:.1f}) on {len(points):,} pts...")

        tree = KDTree(points)
        dists, _ = tree.query(points, k=nb_neighbors + 1)
        mean_dists = dists[:, 1:].mean(axis=1)

        global_mean = mean_dists.mean()
        global_std = mean_dists.std()
        threshold = global_mean + std_ratio * global_std

        inlier_mask = mean_dists <= threshold
        filtered = points[inlier_mask]
        removed = len(points) - len(filtered)
        self.log(f"[FILTER] SOR: removed {removed:,} outliers "
                 f"({100 * removed / max(1, len(points)):.1f}%), "
                 f"kept {len(filtered):,}")
        return filtered

    def _radius_outlier_removal(self, points, radius=0.5, min_neighbors=5):
        """Remove points that have fewer than min_neighbors within radius.
        Requires scipy for KDTree; falls back to no-op if unavailable."""
        if len(points) < min_neighbors or KDTree is None:
            if KDTree is None:
                self.log("[FILTER] scipy not available, skipping ROR")
            return points

        self.log(f"[FILTER] Radius Outlier Removal (r={radius:.2f}m, "
                 f"min_nb={min_neighbors}) on {len(points):,} pts...")

        tree = KDTree(points)
        counts = tree.query_ball_point(points, r=radius, return_length=True)

        # counts includes the point itself, so threshold is min_neighbors + 1
        inlier_mask = counts >= (min_neighbors + 1)
        filtered = points[inlier_mask]
        removed = len(points) - len(filtered)
        self.log(f"[FILTER] ROR: removed {removed:,} sparse points "
                 f"({100 * removed / max(1, len(points)):.1f}%), "
                 f"kept {len(filtered):,}")
        return filtered

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
        """Save accumulated lidar point cloud to PLY with full noise filtering.

        Pipeline:
          1. Merge all scan fragments.
          2. Voxel downsampling (merge nearby duplicates).
          3. Statistical Outlier Removal (SOR) — removes isolated noise.
          4. Radius Outlier Removal (ROR) — removes sparse stray points.
        """
        filepath = filepath or self.ply_output_path or "point_cloud.ply"

        if not self.all_lidar_points:
            self.log("[PLY] No lidar points collected.")
            return None

        self.log("")
        self.log("=" * 60)
        self.log("  POINT CLOUD POST-PROCESSING")
        self.log("=" * 60)

        self.log("[PLY] Merging point cloud from all scans...")
        all_pts = np.concatenate(self.all_lidar_points, axis=0)
        total_raw = len(all_pts)
        self.log(f"[PLY] Raw points: {total_raw:,} "
                 f"(from {len(self.all_lidar_points)} scans)")

        # Step 1: Voxel downsample
        if self.ply_voxel_size > 0:
            all_pts = self._voxel_downsample(all_pts, self.ply_voxel_size)
            self.log(f"[PLY] After voxel downsample ({self.ply_voxel_size}m): "
                     f"{len(all_pts):,} pts "
                     f"({100 * len(all_pts) / max(1, total_raw):.1f}%)")

        if len(all_pts) == 0:
            self.log("[PLY] No points after voxel filtering.")
            return None

        # Step 2: Statistical Outlier Removal
        if self.sor_neighbors > 0:
            all_pts = self._statistical_outlier_removal(
                all_pts,
                nb_neighbors=self.sor_neighbors,
                std_ratio=self.sor_std_ratio
            )

        if len(all_pts) == 0:
            self.log("[PLY] No points after SOR.")
            return None

        # Step 3: Radius Outlier Removal
        if self.ror_min_neighbors > 0:
            all_pts = self._radius_outlier_removal(
                all_pts,
                radius=self.ror_radius,
                min_neighbors=self.ror_min_neighbors
            )

        if len(all_pts) == 0:
            self.log("[PLY] No points after ROR.")
            return None

        final_count = len(all_pts)
        reduction = 100 * (1.0 - final_count / max(1, total_raw))
        self.log(f"[PLY] Final: {final_count:,} pts "
                 f"(removed {reduction:.1f}% of raw data)")

        colors = self._height_to_rgb(all_pts[:, 2])

        out_dir = os.path.dirname(os.path.abspath(filepath))
        os.makedirs(out_dir, exist_ok=True)

        self.log(f"[PLY] Writing {len(all_pts):,} points to: {filepath}")
        with open(filepath, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write("comment Generated by AutonomousExplorer full-coverage scan\n")
            f.write(f"comment Voxel size: {self.ply_voxel_size}m\n")
            f.write(f"comment SOR: k={self.sor_neighbors} std={self.sor_std_ratio}\n")
            f.write(f"comment ROR: r={self.ror_radius} min_nb={self.ror_min_neighbors}\n")
            f.write(f"comment Total scans: {len(self.all_lidar_points)}\n")
            f.write(f"comment Raw points: {total_raw}\n")
            f.write(f"comment Filtered points: {final_count}\n")
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
        self.log("=" * 60)
        return filepath

    # ================================================================
    #  MAIN EXPLORATION LOOP
    # ================================================================

    def explore_loop(self):
        """
        Full-coverage exploration for any building shape.

        Uses frontier-based exploration: the drone always moves toward
        the boundary between known and unknown space, naturally
        discovering aisles, rooms, and corridors without prior knowledge
        of the building layout.

        Phase 1 - Initial 360° scan to map immediate surroundings.
        Phase 2 - Frontier exploration: iteratively fly to the frontier
                  (boundary between known free space and unknown) until
                  no more reachable unknown areas exist.  Works for
                  warehouses, corridors, L-shapes, any geometry.
        Phase 3 - Coverage pass: visit any remaining mapped-but-unvisited
                  cells to ensure dense point-cloud coverage everywhere.
        """

        # ---- Phase 1: Initial full scan ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 1: Initial 360° scan")
        self.log("=" * 60)
        self.visualize_phase_label("PHASE 1: Initial Scan")
        self.scan_full()
        self.visualize_grid_in_sim()

        free_count = int(np.sum(self.grid == EXPLORED))
        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        wall_count = int(np.sum(self.grid == WALL))
        self.log(f"[PHASE 1] After initial scan: {visited_count}/{free_count} "
                 f"cells visited, {wall_count} wall cells")
        self.print_grid(compact=True)

        # ---- Phase 2: Frontier-based exploration ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 2: Frontier exploration (discovering map)")
        self.log("=" * 60)
        self.visualize_phase_label("PHASE 2: Frontier Exploration")

        frontier_steps = 0
        consec_fails = 0
        max_consec_fails = 20
        last_log_step = 0

        while self.is_running_fn():
            self._keepalive_hover()

            # Find nearest reachable frontier
            path = self._find_nearest_frontier()
            if path is None:
                self.log("[PHASE 2] No more frontiers — map fully discovered!")
                break

            frontier_steps += 1
            target_gx, target_gy = path[-1]
            target_wx, target_wy = self.grid_to_world(target_gx, target_gy)

            # Show target in viewport
            self.visualize_current_target(target_wx, target_wy,
                                          bfs_path=path)

            success = self.navigate_to_world(target_wx, target_wy)

            if success:
                consec_fails = 0
                self._wait_for_stable(timeout=2.0, vel_threshold=0.1)
                self.scan_adaptive()

                if frontier_steps % self.viz_update_interval == 0:
                    self.visualize_grid_in_sim()
            else:
                consec_fails += 1
                # Mark this cell as visited so we don't retry it
                if (0 <= target_gx < self.grid_dim_x and
                        0 <= target_gy < self.grid_dim_y and
                        self.grid[target_gx][target_gy] != WALL):
                    self.visited[target_gx][target_gy] = True

                if consec_fails >= max_consec_fails:
                    self.log(f"[PHASE 2] {consec_fails} consecutive failures. "
                             f"Rescanning surroundings...")
                    self.scan_full()
                    consec_fails = 0

            # Log progress periodically
            if frontier_steps - last_log_step >= 5 or (success and frontier_steps - last_log_step >= 3):
                visited_count = int(np.sum(
                    self.visited & (self.grid == EXPLORED)))
                free_count = int(np.sum(self.grid == EXPLORED))
                wall_count = int(np.sum(self.grid == WALL))
                pct = 100 * visited_count / max(1, free_count)
                self.log(f"[PHASE 2] Step {frontier_steps} | "
                         f"Free: {free_count} Walls: {wall_count} | "
                         f"Coverage: {visited_count}/{free_count} ({pct:.0f}%)")
                last_log_step = frontier_steps

        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        free_count = int(np.sum(self.grid == EXPLORED))
        wall_count = int(np.sum(self.grid == WALL))
        pct = 100 * visited_count / max(1, free_count)
        self.log(f"[PHASE 2] Complete. Steps: {frontier_steps}")
        self.log(f"[PHASE 2] Map: {free_count} free, {wall_count} walls")
        self.log(f"[PHASE 2] Coverage: {visited_count}/{free_count} ({pct:.0f}%)")
        self.visualize_grid_in_sim()
        self.print_grid(compact=True)

        # ---- Phase 3: Coverage pass (dense point cloud) ----
        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 3: Coverage pass (visiting remaining areas)")
        self.log("=" * 60)
        self.visualize_phase_label("PHASE 3: Coverage Pass")
        self._keepalive_hover()

        coverage_steps = 0
        consec_fails = 0
        rescan_count = 0
        max_rescans = 5
        target_coverage_pct = 95.0

        while self.is_running_fn():
            # Check coverage
            visited_count = int(np.sum(
                self.visited & (self.grid == EXPLORED)))
            free_count = int(np.sum(self.grid == EXPLORED))
            current_pct = 100 * visited_count / max(1, free_count)
            if current_pct >= target_coverage_pct:
                self.log(f"[PHASE 3] Reached {current_pct:.0f}% coverage — done!")
                break

            self._keepalive_hover()

            # Find nearest unvisited reachable cell
            path = self.find_path_to_nearest_unvisited()
            if path is None:
                rescan_count += 1
                if rescan_count <= max_rescans:
                    self.log(f"[PHASE 3] No path found — rescanning "
                             f"({rescan_count}/{max_rescans})...")
                    self.scan_adaptive()
                    self.update_grid_from_lidar(collect_for_ply=True)
                    continue
                self.log("[PHASE 3] No more reachable unvisited cells.")
                break

            coverage_steps += 1
            target_gx, target_gy = path[-1]
            target_wx, target_wy = self.grid_to_world(target_gx, target_gy)
            self.visualize_current_target(target_wx, target_wy,
                                          bfs_path=path)

            success = self.navigate_to_world(target_wx, target_wy)

            if success:
                consec_fails = 0
                self._wait_for_stable(timeout=2.0, vel_threshold=0.1)
                self.scan_adaptive()
                if coverage_steps % self.viz_update_interval == 0:
                    self.visualize_grid_in_sim()
            else:
                consec_fails += 1
                if (0 <= target_gx < self.grid_dim_x and
                        0 <= target_gy < self.grid_dim_y and
                        self.grid[target_gx][target_gy] != WALL):
                    self.visited[target_gx][target_gy] = True
                if consec_fails >= 20:
                    rescan_count += 1
                    if rescan_count <= max_rescans:
                        self.log(f"[PHASE 3] {consec_fails} consecutive "
                                 f"failures — rescanning "
                                 f"({rescan_count}/{max_rescans})...")
                        consec_fails = 0
                        self.scan_adaptive()
                        self.update_grid_from_lidar(collect_for_ply=True)
                        continue
                    self.log("[PHASE 3] Too many failures — "
                             "stopping coverage pass.")
                    break

            if coverage_steps % 5 == 0:
                self.log(f"[PHASE 3] Step {coverage_steps}: "
                         f"{visited_count}/{free_count} "
                         f"({current_pct:.0f}%)")

        if coverage_steps > 0:
            self.log(f"[PHASE 3] Coverage pass: {coverage_steps} additional stops")
        else:
            self.log("[PHASE 3] No additional coverage needed!")

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
                emg_task = self.client.landAsync()
                self._join_with_timeout(emg_task, timeout=20.0,
                                        task_name="emergency_land")
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
    parser.add_argument("--room-size", type=float, default=200.0,
                        help="Max map size in meters (default: 200, covers most warehouses)")
    parser.add_argument("--cell-size", type=float, default=2.0,
                        help="Grid cell size in meters (default: 2)")
    parser.add_argument("--altitude", type=float, default=2.0,
                        help="Flight altitude in meters (default: 2)")
    parser.add_argument("--speed", type=float, default=2.0,
                        help="Flight speed m/s (default: 2)")
    parser.add_argument("--safe-distance", type=float, default=1.5,
                        help="Min obstacle distance in meters (default: 1.5)")
    parser.add_argument("--auto-room-size", action="store_true",
                        help="Auto-detect building bounds from initial lidar scan")
    parser.add_argument("--auto-room-padding", type=float, default=2.0,
                        help="Extra map padding when auto-detecting (default: 2)")
    parser.add_argument("--scan-rotations", type=int, default=16,
                        help="Rotation steps for 360 scan (default: 16)")
    parser.add_argument("--visit-radius", type=float, default=3.0,
                        help="Radius to mark cells visited (default: 3.0)")
    parser.add_argument("--coverage-spacing", type=float, default=2.5,
                        help="Coverage spacing in meters (default: 2.5)")
    parser.add_argument("--ply-output", type=str, default="point_cloud.ply",
                        help="Output PLY file (default: point_cloud.ply)")
    parser.add_argument("--ply-voxel", type=float, default=0.05,
                        help="Voxel downsample size (default: 0.05)")
    parser.add_argument("--no-ply", action="store_true",
                        help="Disable PLY export")

    # Noise filtering arguments
    parser.add_argument("--lidar-min-range", type=float, default=0.5,
                        help="Min lidar range in meters, closer points discarded (default: 0.5)")
    parser.add_argument("--lidar-max-range", type=float, default=40.0,
                        help="Max lidar range in meters, farther points discarded (default: 40)")
    parser.add_argument("--ply-height-min", type=float, default=None,
                        help="Min Z for PLY points (default: auto, drone_z - 8m)")
    parser.add_argument("--ply-height-max", type=float, default=None,
                        help="Max Z for PLY points (default: auto, drone_z + 3m)")
    parser.add_argument("--sor-neighbors", type=int, default=20,
                        help="Statistical Outlier Removal: k-neighbours (default: 20)")
    parser.add_argument("--sor-std", type=float, default=2.0,
                        help="Statistical Outlier Removal: std ratio (default: 2.0)")
    parser.add_argument("--ror-radius", type=float, default=0.5,
                        help="Radius Outlier Removal: search radius m (default: 0.5)")
    parser.add_argument("--ror-min-neighbors", type=int, default=5,
                        help="Radius Outlier Removal: min neighbours (default: 5)")
    parser.add_argument("--no-sor", action="store_true",
                        help="Disable Statistical Outlier Removal")
    parser.add_argument("--no-ror", action="store_true",
                        help="Disable Radius Outlier Removal")

    # Visualization arguments
    parser.add_argument("--no-viz", action="store_true",
                        help="Disable in-sim debug visualization overlay")
    parser.add_argument("--viz-interval", type=int, default=3,
                        help="Update grid overlay every N waypoints (default: 3)")
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
        lidar_min_range=args.lidar_min_range,
        lidar_max_range=args.lidar_max_range,
        ply_height_min=args.ply_height_min,
        ply_height_max=args.ply_height_max,
        sor_neighbors=0 if args.no_sor else args.sor_neighbors,
        sor_std_ratio=args.sor_std,
        ror_radius=args.ror_radius,
        ror_min_neighbors=0 if args.no_ror else args.ror_min_neighbors,
        visualize_in_sim=not args.no_viz,
        viz_update_interval=args.viz_interval,
    )

    if not args.no_ply:
        explorer.ply_output_path = args.ply_output
        explorer.ply_voxel_size = args.ply_voxel

    explorer.run()
