"""
Utility functions for quaternion math, body-rate computation, and control
window detection.

Conventions:
- Quaternions are [w, x, y, z] (scalar-first).
- ENU → Body quaternion: transforms a vector from the ENU frame to the body
  frame.
- Body-frame axis mapping (RocketPy):
  - Body X → pitch axis     (body_rates_rad_s[0])
  - Body Y → yaw axis       (body_rates_rad_s[1])
  - Body Z → roll longitudinal axis (body_rates_rad_s[2])
- ZYX Euler mapping from ``quaternion_to_euler``:
  - index 0 (φ, x-rotation) → pitch
  - index 1 (θ, y-rotation) → yaw
  - index 2 (ψ, z-rotation) → roll_longitudinal
- Aerospace Euler mapping from ``rocketpy_quaternion_to_aerospace_euler``:
  - roll: rotation about xb = RocketPy z (longitudinal, nose-forward)
  - pitch: elevation of xb above the local ENU horizontal plane
  - yaw: heading of xb, 0 deg North and positive toward East
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
    Converts quaternion [w, x, y, z] to ZYX Euler angles in radians.

    Returns ``(φ, θ, ψ)`` where, for the ENU → Body quaternion used in
    this project (RocketPy body frame: Z = longitudinal axis):

    - φ (index 0, x-rotation) → **pitch** (nose up/down)
    - θ (index 1, y-rotation) → **yaw** (right turn)
    - ψ (index 2, z-rotation) → **roll longitudinal** (around body Z)
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


def quaternion_to_matrix(q):
    """Returns the rotation matrix represented by quaternion [w, x, y, z]."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rocketpy_quaternion_to_aerospace_euler(q, maps_body_to_enu=True):
    """
    Converts a RocketPy attitude quaternion to aerospace roll, pitch, yaw.

    RocketPy's body z-axis is longitudinal and points toward the nose.  This
    project reports aerospace body axes as ``xb = z_rp``, ``yb = x_rp``, and
    ``zb = y_rp`` while keeping ENU as the local navigation frame.

    RocketPy flight states store quaternions as body→ENU, so
    ``maps_body_to_enu`` defaults to ``True``.  Controller reference quaternions
    produced by this project use ENU→body and should pass
    ``maps_body_to_enu=False``.  The returned angles are ``(roll, pitch, yaw)``
    in radians, where pitch is the elevation of ``xb`` above the ENU horizontal
    plane and yaw is 0 at North, positive toward East.
    """
    if np.any(np.isnan(q)):
        return np.nan, np.nan, np.nan

    r = quaternion_to_matrix(q)
    if maps_body_to_enu:
        r_enu_rp = r
        r_enu_body = np.column_stack((
            r_enu_rp[:, 2],  # xb = z_rp
            r_enu_rp[:, 0],  # yb = x_rp
            r_enu_rp[:, 1],  # zb = y_rp
        ))
    else:
        # q maps ENU vectors into RocketPy body coordinates.
        rp_to_body = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        r_body_enu = rp_to_body @ r
        r_enu_body = r_body_enu.T

    xb_enu = r_enu_body[:, 0]
    yb_enu = r_enu_body[:, 1]

    horizontal = np.hypot(xb_enu[0], xb_enu[1])
    pitch = np.arctan2(xb_enu[2], horizontal)
    yaw = np.arctan2(xb_enu[0], xb_enu[1]) if horizontal > 1e-12 else 0.0

    # Reference no-roll frame for the same heading/pitch.  yb points right and
    # zb points down when roll is zero.
    local_down = np.array([0.0, 0.0, -1.0])
    yb_zero = np.cross(local_down, xb_enu)
    norm_yb_zero = np.linalg.norm(yb_zero)
    if norm_yb_zero < 1e-12:
        roll = 0.0
    else:
        yb_zero /= norm_yb_zero
        roll = np.arctan2(
            np.dot(xb_enu, np.cross(yb_zero, yb_enu)),
            np.dot(yb_zero, yb_enu),
        )

    return roll, pitch, yaw


def euler_to_quaternion(roll, pitch, yaw):
    """ZYX Euler angles (roll, pitch, yaw) -> quaternion [w, x, y, z]."""
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def compute_body_rates_from_quaternions(q_history, times):
    """
    Computes body angular rates from a quaternion time history via finite differences.

    Uses the rigid-body kinematic relation for a quaternion **q** representing
    the ENU → Body rotation:

        ω_body = vec( 2 · q* ⊗ q̇ )

    where ``q̇`` is obtained by central finite differences and ``q*`` is the
    quaternion conjugate.  Quaternion sign continuity is enforced before
    differencing (q and −q represent the same rotation).

    Parameters
    ----------
    q_history : array-like, shape (N, 4)
        Quaternion history in [w, x, y, z] order (scalar-first, ENU → Body).
    times : array-like, shape (N,)
        Monotonically increasing timestamps in seconds.

    Returns
    -------
    np.ndarray, shape (N, 3)
        Body angular rates **[ω_pitch, ω_yaw, ω_roll_longitudinal]** in rad/s,
        matching the codebase convention ``body_rates_rad_s[0]=pitch``,
        ``[1]=yaw``, ``[2]=roll_longitudinal``.
        NaN for the first and last samples (central difference not available)
        and wherever the source quaternion is NaN.

    Notes
    -----
    - Convention: ``q_dot = 0.5 * q ⊗ [0, ω_body]``, hence
      ``[0, ω_body] = 2 * conj(q) ⊗ q_dot``.
    - Sign continuity: before differencing, consecutive quaternions are
      re-signed so that ``q[i] · q[i-1] >= 0``.
    - This function is intentionally kept stateless so it can be used for
      both actual and reference quaternion histories.
    """
    n = len(q_history)
    rates = np.full((n, 3), np.nan)
    if n < 3:
        return rates

    q = np.array(q_history, dtype=float)

    # Enforce quaternion sign continuity
    for i in range(1, n):
        if np.any(np.isnan(q[i])) or np.any(np.isnan(q[i - 1])):
            continue
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]

    # Central finite difference for q_dot, then compute body rates
    for i in range(1, n - 1):
        if (np.any(np.isnan(q[i]))
                or np.any(np.isnan(q[i - 1]))
                or np.any(np.isnan(q[i + 1]))):
            continue
        dt = times[i + 1] - times[i - 1]
        if dt > 0:
            q_dot = (q[i + 1] - q[i - 1]) / dt
            omega_quat = 2.0 * quaternion_multiply(
                quaternion_conjugate(q[i]), q_dot
            )
            rates[i] = omega_quat[1:4]  # [ωx, ωy, ωz]

    return rates


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
