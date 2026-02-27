"""
Drone position, yaw, orientation.
Resilient to RPC timeouts and IOLoop errors — recovery + cached fallback.
"""

import math


def _try_get_position_from_state(self, state, use_kinematics_estimated=False):
    """Extract (x,y,z) from state object."""
    if use_kinematics_estimated:
        p = state.kinematics_estimated.position
    else:
        p = state.position
    return p.x_val, p.y_val, p.z_val


def get_position(self):
    """Get current position as (x, y, z) tuple.
    Resilient: RPC recovery on timeout/IOLoop, fallback to getMultirotorState,
    last-resort cached position to prevent crash.
    """
    if self.use_mock:
        state = self.client.getMultirotorState()
        p = state.kinematics_estimated.position
        result = p.x_val, p.y_val, p.z_val
        self._last_known_position = result
        return result

    for attempt in range(2):
        try:
            try:
                state = self.client.simGetGroundTruthKinematics()
                result = _try_get_position_from_state(self, state, use_kinematics_estimated=False)
            except Exception as e1:
                self._log_throttled_error(
                    "get_position_ground_truth",
                    "simGetGroundTruthKinematics failed, fallback to estimated state",
                    e1
                )
                state = self.client.getMultirotorState()
                result = _try_get_position_from_state(self, state, use_kinematics_estimated=True)

            self._last_known_position = result
            return result

        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("get_position")
                continue
            self._log_throttled_error(
                "get_position",
                "get_position RPC failed after retry",
                e
            )
            if self._last_known_position is not None:
                self.log("[WARN] Using cached position (RPC degraded).")
                return self._last_known_position
            if self.start_position is not None:
                p = self.start_position
                return p.x_val, p.y_val, p.z_val
            return 0.0, 0.0, float(self.target_altitude)


def get_yaw(self):
    """Get current yaw angle in radians.
    Resilient: same RPC recovery and fallback strategy as get_position.
    """
    def _yaw_from_orientation(o):
        siny = 2.0 * (o.w_val * o.z_val + o.x_val * o.y_val)
        cosy = 1.0 - 2.0 * (o.y_val ** 2 + o.z_val ** 2)
        return math.atan2(siny, cosy)

    if self.use_mock:
        state = self.client.getMultirotorState()
        o = state.kinematics_estimated.orientation
        result = _yaw_from_orientation(o)
        self._last_known_yaw = result
        return result

    for attempt in range(2):
        try:
            try:
                state = self.client.simGetGroundTruthKinematics()
                o = state.orientation
            except Exception as e1:
                self._log_throttled_error(
                    "get_yaw_ground_truth",
                    "simGetGroundTruthKinematics failed for yaw, fallback to estimated",
                    e1
                )
                state = self.client.getMultirotorState()
                o = state.kinematics_estimated.orientation

            result = _yaw_from_orientation(o)
            self._last_known_yaw = result
            return result

        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("get_yaw")
                continue
            self._log_throttled_error("get_yaw", "get_yaw RPC failed after retry", e)
            return self._last_known_yaw


def _get_orientation_quaternion(self):
    """Get current orientation as (w, x, y, z) quaternion using ground truth.
    Resilient: same RPC recovery and fallback strategy.
    """
    if self.use_mock:
        state = self.client.getMultirotorState()
        o = state.kinematics_estimated.orientation
        return o.w_val, o.x_val, o.y_val, o.z_val

    for attempt in range(2):
        try:
            try:
                state = self.client.simGetGroundTruthKinematics()
                o = state.orientation
            except Exception as e1:
                self._log_throttled_error(
                    "get_orientation_ground_truth",
                    "Ground-truth orientation failed, fallback to estimated",
                    e1
                )
                state = self.client.getMultirotorState()
                o = state.kinematics_estimated.orientation

            return o.w_val, o.x_val, o.y_val, o.z_val

        except Exception as e:
            if self._is_rpc_recoverable_error(e) and attempt == 0:
                self._recover_rpc_client("get_orientation")
                continue
            self._log_throttled_error(
                "get_orientation",
                "get_orientation RPC failed, using identity quaternion",
                e
            )
            return 1.0, 0.0, 0.0, 0.0
