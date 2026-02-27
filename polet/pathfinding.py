"""
BFS path planning, frontier discovery, path simplification.
"""

from collections import deque
import numpy as np

from polet.constants import UNEXPLORED, EXPLORED, WALL


def _is_passable(self, gx, gy):
    """Check if a grid cell can be traversed."""
    if self.grid[gx][gy] == EXPLORED:
        return True
    if self.visited[gx][gy]:
        return True
    return False


def bfs_path_to(self, start_gx, start_gy, target_gx, target_gy):
    """BFS from (start) to (target) through passable cells."""
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
                if abs(dgx) + abs(dgy) == 2:
                    if (self.grid[gx + dgx][gy] == WALL or
                            self.grid[gx][gy + dgy] == WALL):
                        continue
                seen.add(nb)
                parent[nb] = current
                queue.append(nb)

    return None


def find_path_to_nearest_unvisited(self):
    """BFS to nearest EXPLORED (free) cell that has not been visited."""
    x, y, _ = self.get_position()
    start = self.world_to_grid(x, y)

    queue = deque([start])
    seen = {start}
    parent = {start: None}

    while queue:
        current = queue.popleft()
        gx, gy = current

        if (self.grid[gx][gy] == EXPLORED and
                not self.visited[gx][gy]):
            path = []
            node = current
            while node is not None:
                path.append(node)
                node = parent[node]
            path.reverse()
            return path

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
    """BFS to nearest frontier cell from drone position."""
    x, y, _ = self.get_position()
    start = self.world_to_grid(x, y)

    queue = deque([start])
    seen = {start}
    parent = {start: None}

    while queue:
        current = queue.popleft()
        gx, gy = current

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
    """Count UNEXPLORED cells within given grid-cell radius."""
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
    """Check if a straight line between two grid cells crosses any WALL cell."""
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
            return False

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
    """Remove redundant intermediate waypoints from a BFS grid path."""
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
