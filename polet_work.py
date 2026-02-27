"""
Autonomous Full-Coverage Explorer for AirSim Drones.

Точка входа: делегирует в пакет polet.
Используйте: python polet_work.py [аргументы]
Или:        from polet import AutonomousExplorer

Разбито на модули для уменьшения контекста:
  polet/constants.py     - константы, обёртки AirSim
  polet/connection.py    - подключение, RPC recovery
  polet/async_utils.py   - keepalive, join, wait_for_stable
  polet/grid.py          - сетка, координаты, auto room
  polet/drone_state.py   - position, yaw, orientation
  polet/visualization.py - траектория, debug overlay
  polet/lidar.py         - lidar в world coords
  polet/grid_mapping.py  - обновление occupancy grid
  polet/scanning.py      - 360° scan (full, adaptive)
  polet/waypoints.py     - perimeter, interior, obstacles
  polet/pathfinding.py   - BFS, frontiers, simplify
  polet/navigation.py    - navigate_to_cell/world
  polet/flight.py        - takeoff, return_home
  polet/ply_export.py    - PLY save
  polet/explorer.py      - AutonomousExplorer
  polet/main.py          - argparse, run
"""

from polet.main import main
from polet import AutonomousExplorer

__all__ = ["AutonomousExplorer", "main"]

if __name__ == "__main__":
    main()
