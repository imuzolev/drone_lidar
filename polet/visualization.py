"""
Trajectory recording and in-sim debug visualization.
"""

try:
    import airsim
except ImportError:
    airsim = None

from polet.constants import WALL, EXPLORED


def record_trajectory(self):
    """Record position (no red line drawing in UE5)."""
    x, y, z = self.get_position()

    if self.use_mock or airsim is None:
        self.trajectory_points.append((x, y, z))
        if len(self.trajectory_points) > self.max_traj_points:
            self.trajectory_points = self.trajectory_points[-self.max_traj_points:]
        return

    point = airsim.Vector3r(x, y, z)
    self.trajectory_points.append(point)
    if len(self.trajectory_points) > self.max_traj_points:
        self.trajectory_points = self.trajectory_points[-self.max_traj_points:]


def visualize_grid_in_sim(self, duration=20.0):
    """Draw coloured grid overlay in UE viewport."""
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
    """Draw a planned waypoint path and its markers in UE viewport."""
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
    """Highlight current navigation target and optional BFS route."""
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
