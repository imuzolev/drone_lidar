"""
Takeoff and return home.
"""

import time
import math


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
    self.get_position()
    self.get_yaw()
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

    self.log("[FLIGHT] Landing...")
    try:
        land_task = self.client.landAsync()
        self._join_with_timeout(land_task, timeout=30.0,
                                task_name="landing")
    except Exception as e:
        self.log(f"[FLIGHT] Landing exception (non-critical): {e}")
    self.record_trajectory()
    self.log("[FLIGHT] Landed successfully.")
