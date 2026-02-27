"""
AutonomousExplorer main class — composes all modules.
"""

import time
import os
import logging
import threading
import traceback
import numpy as np

from polet.constants import (
    UNEXPLORED, EXPLORED, WALL,
    _SerializedAirSimClient,
)

# Import and attach module methods
from polet import connection
from polet import async_utils
from polet import grid
from polet import drone_state
from polet import visualization
from polet import lidar
from polet import grid_mapping
from polet import scanning
from polet import waypoints
from polet import pathfinding
from polet import navigation
from polet import flight
from polet import grid_display
from polet import ply_export


class AutonomousExplorer:
    """
    Full-coverage autonomous drone explorer.
    """

    def __init__(self, client=None, use_mock=False, room_size=20.0,
                 cell_size=2.0, altitude=3.0, speed=1.5, safe_distance=1.0,
                 require_sim=True, auto_room_size=False, auto_room_padding=1.0,
                 scan_rotations=12, visit_radius=2.5, coverage_spacing=3.0,
                 lidar_min_range=0.5, lidar_max_range=40.0,
                 ply_height_min=None, ply_height_max=None,
                 sor_neighbors=20, sor_std_ratio=2.0,
                 ror_radius=0.5, ror_min_neighbors=5,
                 scan_pause_after_stable=0.15, scan_stable_vel_threshold=0.1,
                 use_adaptive_scan=True,
                 visualize_in_sim=True, viz_update_interval=3,
                 log_file=None, log_level="INFO"):
        self._logger = self._create_logger(log_file, log_level)
        self._last_error_log_ts = {}
        self.log = self._log
        self.is_running_fn = lambda: True
        self._rpc_lock = threading.RLock()

        self.use_mock = use_mock
        self.client = client
        self.external_client = client is not None
        self.require_sim = require_sim

        if not self.external_client:
            self._connect(use_mock)
        else:
            self.client = _SerializedAirSimClient(self.client, self._rpc_lock)

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

        self.target_altitude = -abs(altitude)
        self.safe_distance = safe_distance
        self.speed = speed
        self.auto_room_size = auto_room_size
        self.auto_room_padding = max(0.0, float(auto_room_padding))

        self.scan_rotations = max(4, int(scan_rotations))
        self.visit_radius = max(self.cell_size, float(visit_radius))
        self.coverage_spacing = max(self.cell_size, float(coverage_spacing))

        self.lidar_min_range = max(0.1, float(lidar_min_range))
        self.lidar_max_range = max(1.0, float(lidar_max_range))

        self.ply_height_min = ply_height_min
        self.ply_height_max = ply_height_max
        self.sor_neighbors = max(1, int(sor_neighbors))
        self.sor_std_ratio = max(0.1, float(sor_std_ratio))
        self.ror_radius = max(0.01, float(ror_radius))
        self.ror_min_neighbors = max(1, int(ror_min_neighbors))

        self.scan_pause_after_stable = max(0.0, float(scan_pause_after_stable))
        self.scan_stable_vel_threshold = max(0.01, float(scan_stable_vel_threshold))
        self.use_adaptive_scan = bool(use_adaptive_scan)

        self.start_position = None
        self.trajectory_points = []

        self.all_lidar_points = []
        self.ply_output_path = None
        self.ply_voxel_size = 0.05

        self.visualize_in_sim = bool(visualize_in_sim)
        self.viz_update_interval = max(1, int(viz_update_interval))
        self._last_traj_plot_ts = 0.0
        self.traj_plot_interval_s = 1.0
        self.max_traj_points = 1200
        self.traj_plot_window = 200
        self.lightweight_scan_mode = False

        self._last_known_position = None
        self._last_known_yaw = 0.0

    def _create_logger(self, log_file, log_level):
        logger = logging.getLogger(f"autonomous_explorer_{id(self)}")
        logger.setLevel(getattr(logging, str(log_level).upper(), logging.INFO))
        logger.propagate = False

        if logger.handlers:
            return logger

        if not log_file:
            ts = time.strftime("%Y%m%d_%H%M%S")
            log_dir = os.path.join(os.getcwd(), "logs")
            os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, f"exploration_{ts}.log")
        else:
            out_dir = os.path.dirname(os.path.abspath(log_file))
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)

        fh = logging.FileHandler(log_file, encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            "%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        ))
        logger.addHandler(fh)

        self._log_file = log_file
        return logger

    def _log(self, message):
        text = str(message)
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {text}")

        upper = text.upper()
        if "[ERROR]" in upper:
            lvl = logging.ERROR
        elif "[WARN]" in upper or "[TIMEOUT]" in upper:
            lvl = logging.WARNING
        else:
            lvl = logging.INFO
        try:
            self._logger.log(lvl, text)
        except Exception:
            pass

    def _log_throttled_error(self, key, message, exc=None, interval_s=5.0):
        now = time.time()
        prev = self._last_error_log_ts.get(key, 0.0)
        if now - prev < interval_s:
            return
        self._last_error_log_ts[key] = now
        if exc is None:
            self.log(f"[WARN] {message}")
            return
        self.log(f"[WARN] {message}: {exc}")
        try:
            self._logger.error("Traceback for %s:\n%s", key, traceback.format_exc())
        except Exception:
            pass

    def explore_loop(self):
        """Full-coverage exploration for any building shape."""
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

            path = self._find_nearest_frontier()
            if path is None:
                self.log("[PHASE 2] No more frontiers — map fully discovered!")
                break

            frontier_steps += 1
            target_gx, target_gy = path[-1]
            target_wx, target_wy = self.grid_to_world(target_gx, target_gy)

            self.visualize_current_target(target_wx, target_wy, bfs_path=path)

            success = self.navigate_to_world(target_wx, target_wy)

            if success:
                consec_fails = 0
                self._wait_for_stable(timeout=2.0, vel_threshold=0.1)
                self.scan_adaptive()

                if frontier_steps % self.viz_update_interval == 0:
                    self.visualize_grid_in_sim()
            else:
                consec_fails += 1
                if (0 <= target_gx < self.grid_dim_x and
                        0 <= target_gy < self.grid_dim_y and
                        self.grid[target_gx][target_gy] != WALL):
                    self.visited[target_gx][target_gy] = True

                if consec_fails >= max_consec_fails:
                    self.log(f"[PHASE 2] {consec_fails} consecutive failures. "
                             f"Rescanning surroundings...")
                    self.scan_full()
                    consec_fails = 0

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

        self.log("")
        self.log("=" * 60)
        self.log("  PHASE 3: Coverage pass (visiting remaining areas)")
        self.log("=" * 60)
        if self.visualize_in_sim:
            self.log("[STABILITY] Phase 3: disabling UE debug visualization.")
        self.visualize_in_sim = False
        self.lightweight_scan_mode = True
        self.visualize_phase_label("PHASE 3: Coverage Pass")
        self._keepalive_hover()

        coverage_steps = 0
        consec_fails = 0
        rescan_count = 0
        max_rescans = 5
        target_coverage_pct = 95.0

        while self.is_running_fn():
            visited_count = int(np.sum(
                self.visited & (self.grid == EXPLORED)))
            free_count = int(np.sum(self.grid == EXPLORED))
            current_pct = 100 * visited_count / max(1, free_count)
            if current_pct >= target_coverage_pct:
                self.log(f"[PHASE 3] Reached {current_pct:.0f}% coverage — done!")
                break

            self._keepalive_hover()

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
            self.visualize_current_target(target_wx, target_wy, bfs_path=path)

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
        self.lightweight_scan_mode = False

        visited_count = int(np.sum(self.visited & (self.grid == EXPLORED)))
        free_count = int(np.sum(self.grid == EXPLORED))
        wall_count = int(np.sum(self.grid == WALL))
        coverage = 100 * visited_count / max(1, free_count)

        self.log("")
        self.log("=" * 60)
        self.log("  EXPLORATION COMPLETE")
        self.log(f"  Free cells:      {free_count}")
        self.log(f"  Wall cells:     {wall_count}")
        self.log(f"  Visited:        {visited_count}/{free_count} "
                 f"({coverage:.0f}% coverage)")
        self.log(f"  Trajectory pts: {len(self.trajectory_points)}")
        self.log(f"  Lidar scans:    {len(self.all_lidar_points)}")
        self.log("=" * 60)

        self.print_grid(compact=True)

    def run(self, log_callback=None, running_check=None):
        """Full exploration lifecycle."""
        if log_callback:
            self.log = log_callback
        if running_check:
            self.is_running_fn = running_check

        try:
            if hasattr(self, "_log_file"):
                self.log(f"[INFO] Diagnostic log file: {self._log_file}")
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
            tb = traceback.format_exc()
            self.log(tb.rstrip())
            try:
                self._logger.error("Unhandled exception traceback:\n%s", tb)
            except Exception:
                pass
            try:
                self.log("[EMERGENCY] Attempting emergency landing...")
                emg_task = self.client.landAsync()
                self._join_with_timeout(emg_task, timeout=20.0,
                                        task_name="emergency_land")
            except Exception as emg_e:
                self.log(f"[ERROR] Emergency landing failed: {emg_e}")
        finally:
            if not self.external_client:
                for cleanup_attempt in range(2):
                    try:
                        self.client.armDisarm(False)
                        self.client.enableApiControl(False)
                        break
                    except Exception as disarm_e:
                        if self._is_rpc_recoverable_error(disarm_e) and cleanup_attempt == 0:
                            self._recover_rpc_client("cleanup")
                            continue
                        self.log(f"[WARN] Cleanup failed (arm/api control): {disarm_e}")
                        break
            self.log("[INFO] Exploration session ended.")


# Attach methods from modules
def _attach(module, *names):
    for name in names:
        if hasattr(module, name):
            setattr(AutonomousExplorer, name, getattr(module, name))

_attach(connection, "_connect", "_is_ioloop_running_error", "_is_rpc_timeout_error",
       "_is_rpc_recoverable_error", "_recover_rpc_client")
_attach(async_utils, "_keepalive_hover", "_join_with_timeout", "_move_timeout", "_wait_for_stable")
_attach(grid, "_configure_grid_rect", "_auto_configure_room_from_lidar", "_recalibrate_grid_from_walls",
       "world_to_grid", "grid_to_world")
_attach(drone_state, "get_position", "get_yaw", "_get_orientation_quaternion")
_attach(visualization, "record_trajectory", "visualize_grid_in_sim", "visualize_planned_path",
        "visualize_current_target", "visualize_phase_label")
_attach(lidar, "get_lidar_points_world", "get_distance_in_direction")
_attach(grid_mapping, "update_grid_from_lidar", "_mark_ray_explored")
_attach(scanning, "scan_full", "mark_cells_visited", "_unexplored_ratio_at", "scan_adaptive")
_attach(waypoints, "is_location_reachable", "is_location_safe", "_generate_perimeter_waypoints",
        "_generate_interior_waypoints", "generate_coverage_waypoints", "_generate_fill_waypoints",
        "_detect_internal_obstacles", "_obstacle_centroid_and_radius",
        "_generate_obstacle_circumnavigation_waypoints")
_attach(pathfinding, "_is_passable", "bfs_path_to", "find_path_to_nearest_unvisited",
        "_find_nearest_frontier", "_count_nearby_unexplored", "_grid_line_clear", "_simplify_path")
_attach(navigation, "_adaptive_speed", "navigate_to_cell", "navigate_to_world")
_attach(flight, "takeoff", "return_home_smooth")
_attach(grid_display, "print_grid")
_attach(ply_export, "_voxel_downsample", "_statistical_outlier_removal", "_radius_outlier_removal",
        "_height_to_rgb", "save_point_cloud_ply")
