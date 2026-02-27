"""
Autonomous Full-Coverage Explorer for AirSim Drones.

Split from polet_work.py for reduced context usage.
Use: from polet import AutonomousExplorer
"""

from polet.explorer import AutonomousExplorer
from polet.constants import UNEXPLORED, EXPLORED, WALL

__all__ = ["AutonomousExplorer", "UNEXPLORED", "EXPLORED", "WALL"]
