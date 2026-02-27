"""
AirSim connection and RPC recovery.
"""

import os
import time
import traceback

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
        try:
            import importlib.util
            _spec = importlib.util.spec_from_file_location(
                "mock_airsim",
                os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "mock_airsim.py")
            )
            mock_airsim = importlib.util.module_from_spec(_spec)
            _spec.loader.exec_module(mock_airsim)
        except Exception:
            mock_airsim = None

from polet.constants import _SerializedAirSimClient


def _connect(self, use_mock):
    """Connect to AirSim or fall back to mock."""
    if not use_mock and airsim is not None:
        try:
            self.log("[CONNECT] Connecting to AirSim...")
            raw_client = airsim.MultirotorClient()
            raw_client.confirmConnection()
            self.client = _SerializedAirSimClient(raw_client, self._rpc_lock)
            self.log("[CONNECT] Connected to AirSim.")
            self.client.reset()
            time.sleep(1.0)
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            self.log("[CONNECT] Simulator reset and ready.")
            self.use_mock = False
            return
        except Exception as e:
            self.log(f"[ERROR] AirSim: {e}")
            try:
                self._logger.error("AirSim connection traceback:\n%s",
                                   traceback.format_exc())
            except Exception:
                pass
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
    self.client = _SerializedAirSimClient(mock_airsim.MultirotorClient(), self._rpc_lock)
    self.client.confirmConnection()


def _is_ioloop_running_error(self, exc):
    return isinstance(exc, RuntimeError) and "IOLoop is already running" in str(exc)


def _is_rpc_timeout_error(self, exc):
    """Check if exception is an RPC timeout (msgpackrpc or similar)."""
    s = str(exc).lower()
    name = type(exc).__name__
    return "timed out" in s or "timeout" in name


def _is_rpc_recoverable_error(self, exc):
    """Check if RPC client should be reconnected (IOLoop conflict or timeout)."""
    return self._is_ioloop_running_error(exc) or self._is_rpc_timeout_error(exc)


def _recover_rpc_client(self, context="rpc"):
    """Reconnect RPC session after msgpack loop corruption."""
    if self.use_mock or airsim is None:
        return False
    if self.external_client:
        self.log(f"[RPC] {context}: external client cannot be auto-reconnected.")
        return False

    self.log(f"[RPC] {context}: reconnecting AirSim client after RPC failure...")
    try:
        raw_client = airsim.MultirotorClient()
        raw_client.confirmConnection()
        recovered = _SerializedAirSimClient(raw_client, self._rpc_lock)
        recovered.enableApiControl(True)
        recovered.armDisarm(True)
        self.client = recovered
        self.log("[RPC] Client session recovered.")
        return True
    except Exception as reconnect_exc:
        self.log(f"[RPC] Reconnect failed: {reconnect_exc}")
        try:
            self._logger.error("RPC reconnect traceback:\n%s", traceback.format_exc())
        except Exception:
            pass
        return False
