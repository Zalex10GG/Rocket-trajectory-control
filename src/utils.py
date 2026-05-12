"""
Utility functions for quaternion math and control window detection.

Conventions:
- Quaternions are [w, x, y, z] (scalar-first).
- Control-active window: samples where the controller diagnostics report
  ``control_active=True``.  This is the authoritative signal, not nonzero
  deltas alone.
- Ascent window: from control activation to apogee (may include post-cutoff
  coasting when dynamic pressure drops).
"""

import numpy as np


def quaternion_conjugate(q):
    """Returns the conjugate of a quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_multiply(q1, q2):
    """Multiplies two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quaternion_from_vectors(v_from, v_to):
    """
    Returns a quaternion representing the rotation from v_from to v_to.
    Both vectors must be normalized.
    """
    norm_from = np.linalg.norm(v_from)
    norm_to = np.linalg.norm(v_to)
    
    if norm_from < 1e-9:
        raise ValueError("v_from vector is too small (near zero norm).")
    if norm_to < 1e-9:
        raise ValueError("v_to vector is too small (near zero norm).")

    v_from = v_from / norm_from
    v_to = v_to / norm_to
    
    dot = np.dot(v_from, v_to)
    if dot > 0.999999:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -0.999999:
        # 180 degree rotation around any orthogonal axis
        axis = np.array([0, 1, 0])
        if abs(v_from[1]) > 0.9:
            axis = np.array([1, 0, 0])
        axis = np.cross(v_from, axis)
        axis = axis / np.linalg.norm(axis)
        return np.array([0.0, axis[0], axis[1], axis[2]])
    
    axis = np.cross(v_from, v_to)
    s = np.sqrt((1 + dot) * 2)
    inv_s = 1 / s
    return np.array([s * 0.5, axis[0] * inv_s, axis[1] * inv_s, axis[2] * inv_s])

def quaternion_to_euler(q):
    """
    Converts quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw) in radians.
    Standard ZYX convention.
    """
    w, x, y, z = q
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.sign(sinp) * np.pi / 2 # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def get_control_window_indices(flight_history, controller_state=None):
    """
    Identifies indices for the active-control and ascent windows.

    The active-control window is defined from the first sample where the
    controller diagnostics report ``control_active=True`` to the last such
    sample.  If no diagnostics are available, falls back to nonzero deltas.

    The ascent window extends from the first active-control sample to apogee.

    Parameters
    ----------
    flight_history : list[dict]
        Flight state records.
    controller_state : dict, optional
        Controller state with ``_diagnostics`` list for authoritative
        active-control detection.

    Returns
    -------
    tuple[int, int]
        ``(ctrl_start_idx, apogee_idx)`` — active-control start and apogee end.
    """
    if not flight_history:
        return 0, 0

    pos_z = np.array([s['position_enu_m'][2] for s in flight_history])
    times = np.array([s['time_s'] for s in flight_history])

    # Determine active-control start from diagnostics if available
    ctrl_start_idx = 0
    diag = controller_state.get("_diagnostics", []) if controller_state else []
    if diag:
        active_times = [d["time_s"] for d in diag if d.get("control_active", False)]
        if active_times:
            first_active_t = min(active_times)
            # Find the flight_history index closest to first_active_t
            ctrl_start_idx = int(np.argmin(np.abs(times - first_active_t)))

    if ctrl_start_idx == 0:
        # Fallback: use nonzero deltas
        deltas = np.array([s['deltas'] for s in flight_history])
        ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
        ctrl_active_indices = np.where(ctrl_active_mask)[0]
        if len(ctrl_active_indices) > 0:
            ctrl_start_idx = ctrl_active_indices[0]

    # Apogee is max altitude
    apogee_idx = int(np.argmax(pos_z))

    # Sanity check: ensure start is before end
    if ctrl_start_idx >= apogee_idx:
        ctrl_start_idx = 0

    return int(ctrl_start_idx), int(apogee_idx)


def get_active_control_window_indices(flight_history, controller_state=None):
    """
    Returns the start and end indices of the strictly active-control window.

    Unlike ``get_control_window_indices``, this does NOT extend to apogee.
    It returns the first and last samples where the controller was actively
    commanding control (based on diagnostics or nonzero deltas).

    Parameters
    ----------
    flight_history : list[dict]
        Flight state records.
    controller_state : dict, optional
        Controller state with ``_diagnostics``.

    Returns
    -------
    tuple[int, int]
        ``(active_start_idx, active_end_idx)``.
    """
    if not flight_history:
        return 0, 0

    times = np.array([s['time_s'] for s in flight_history])

    # Prefer diagnostics for authoritative active window
    diag = controller_state.get("_diagnostics", []) if controller_state else []
    if diag:
        active_times = [d["time_s"] for d in diag if d.get("control_active", False)]
        if active_times:
            first_t = min(active_times)
            last_t = max(active_times)
            start_idx = int(np.argmin(np.abs(times - first_t)))
            end_idx = int(np.argmin(np.abs(times - last_t)))
            return start_idx, end_idx

    # Fallback: nonzero deltas
    deltas = np.array([s['deltas'] for s in flight_history])
    ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
    ctrl_active_indices = np.where(ctrl_active_mask)[0]
    if len(ctrl_active_indices) > 0:
        return int(ctrl_active_indices[0]), int(ctrl_active_indices[-1])

    return 0, 0
