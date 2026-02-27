"""
Constants and AirSim client wrappers.
"""

import threading

try:
    import airsim
except ImportError:
    airsim = None

# Grid cell states
UNEXPLORED = 0
EXPLORED = 1   # Lidar ray passed through (known free space)
WALL = 2


class _SerializedAirSimTask:
    """Serialize Async.join() to protect msgpackrpc IOLoop state."""

    def __init__(self, task, rpc_lock):
        self._task = task
        self._rpc_lock = rpc_lock

    def join(self):
        with self._rpc_lock:
            return self._task.join()

    def __getattr__(self, name):
        return getattr(self._task, name)


class _SerializedAirSimClient:
    """Serialize all client calls through a shared re-entrant lock."""

    def __init__(self, client, rpc_lock):
        self._client = client
        self._rpc_lock = rpc_lock

    def __getattr__(self, name):
        attr = getattr(self._client, name)
        if not callable(attr):
            return attr

        def wrapped(*args, **kwargs):
            with self._rpc_lock:
                result = attr(*args, **kwargs)
            if name.endswith("Async") and hasattr(result, "join"):
                return _SerializedAirSimTask(result, self._rpc_lock)
            return result

        return wrapped
