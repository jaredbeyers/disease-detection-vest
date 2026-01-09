# tof_wrapper.py
import os
import ctypes

_here = os.path.dirname(__file__)
_lib = ctypes.CDLL(os.path.join(_here, "libdistance.so"))

# Function signatures
_lib.tof_init.restype = ctypes.c_int
_lib.tof_get_avg_distance_mm.restype = ctypes.c_double
_lib.tof_shutdown.restype = None

def get_distance_m():
    """
    Initialize if needed, then return average distance in meters.
    Returns None if there's an error / no valid target.
    """
    rc = _lib.tof_init()
    if rc != 0:
        print(f"[tof_wrapper] init failed with status {rc}")
        return None

    d_mm = _lib.tof_get_avg_distance_mm()
    if d_mm < 0:
        return None
    return d_mm / 1000.0

def shutdown():
    _lib.tof_shutdown()
