"""
Grid console printing.
"""

import numpy as np

from polet.constants import UNEXPLORED, EXPLORED, WALL


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
