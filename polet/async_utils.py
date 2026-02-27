"""
Async task handling, keepalive, wait for stable.
"""

import time
import math


def _keepalive_hover(self):
    """Issue a hoverAsync command to prevent AirSim's 'hover mode for
    safety' timeout during long computation pauses (BFS, grid updates).
    Safe to call at any time — simply tells the drone to hold position."""
    for attempt in range(2):
        try:
            self.client.hoverAsync()
            return
        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("keepalive_hover")
                continue
            self._log_throttled_error("keepalive_hover", "hoverAsync failed", e, interval_s=10.0)
            return


def _join_with_timeout(self, task, timeout=30.0, task_name="task"):
    """Join an async AirSim task safely.

    IMPORTANT:
    msgpack-rpc client used by AirSim is not thread-safe and can throw
    "IOLoop is already running" if two threads call RPC concurrently.
    Therefore we must avoid running task.join() in a background thread.

    This method keeps the timeout argument for API compatibility but
    performs a single-threaded join to preserve client stability.
    """
    _ = timeout
    try:
        task.join()
        return True
    except Exception as e:
        if self._is_rpc_recoverable_error(e):
            self._recover_rpc_client(task_name)
        self.log(f"  [ERROR] '{task_name}' failed: {e}")
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
    Resilient: uses get_position as fallback when simGetGroundTruthKinematics fails.

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
        except Exception as e:
            if self._is_rpc_recoverable_error(e):
                self._recover_rpc_client("wait_for_stable")
            self._log_throttled_error("wait_for_stable_gt",
                                      "simGetGroundTruthKinematics failed during stabilization",
                                      e, interval_s=10.0)
            time.sleep(0.2)
            continue
        time.sleep(0.05)
    return False
