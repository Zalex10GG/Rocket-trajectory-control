"""
Closed-loop fin controller for rocket trajectory tracking.

Implements:
- Outer-loop PD guidance with feedforward reference acceleration.
- Wind disturbance feedforward compensation via local wind velocity.
- Inner-loop attitude PID with back-calculation anti-windup.
- Dynamic-pressure-based control cutoff (fin authority loss).
- Diagnostic tracking of dynamic pressure, airspeed, wind speed, saturation.
- Idempotent controller update per effective timestamp.
- q-bar scheduled deflection authority guardrail.

Conventions:
- All vectors in local ENU (launch pad = origin).
- Quaternions [w, x, y, z] for ENU-to-Body rotation.
- Gravity compensation: ``+ [0, 0, g]`` in ENU (upward bias to counteract
  Earth's gravitational acceleration, which points ``[0, 0, -g]`` in ENU).
- Velocity error ``e_vel`` is computed in the inertial frame (ref - vel_inertial),
  NOT with airspeed. Wind is compensated separately as a feedforward disturbance.

References:
- NotebookLM V1-TFG (2026-05-11): wind-compensation-anti-windup-dynamic-pressure
- Åström & Hägglund, PID Controllers (1995), Ch. 3 — back-calculation anti-windup.
"""

import numpy as np
import src.utils as utils


# Tolerance for timestamp deduplication: callbacks within this dt are treated
# as the same effective control sample.
_TIME_TOL = 1e-9


def _compute_qbar_authority_limit(q_dynamic, config):
    """
    Compute the q-bar scheduled deflection authority limit.

    Authority follows a trapezoidal schedule:
    - low q-bar: reduced authority because fins have little effect;
    - nominal q-bar: full configured authority;
    - excessive q-bar: reduced authority to avoid large control drag just
      after burnout while the rocket is still very fast.

    Parameters
    ----------
    q_dynamic : float
        Current dynamic pressure (Pa).
    config : Config
        Execution configuration with q-bar scheduling parameters.

    Returns
    -------
    float
        Effective maximum deflection (rad) for the current q-bar.
    """
    qbar_min = getattr(config, 'qbar_min_authority_pa', 0.0)
    qbar_full = getattr(config, 'qbar_full_authority_pa', 0.0)
    qbar_high = getattr(config, 'qbar_high_authority_pa', float("inf"))
    delta_max = getattr(config, '_delta_max_rad_from_toml', 0.3490658503988659)
    delta_min = getattr(config, 'delta_max_qbar_min_rad', 0.0)
    delta_high = getattr(config, 'delta_max_qbar_high_rad', delta_max)

    # Guard: if scheduling is disabled, return full authority.
    if qbar_full <= qbar_min:
        return delta_max

    if q_dynamic <= qbar_min:
        return delta_min

    if q_dynamic <= qbar_full:
        # Linear ramp from delta_min to delta_max
        frac = (q_dynamic - qbar_min) / (qbar_full - qbar_min)
        return delta_min + frac * (delta_max - delta_min)

    # If no high-q reduction is configured, hold full authority above qbar_full.
    if qbar_high <= qbar_full:
        return delta_high
    if q_dynamic >= qbar_high:
        return delta_high

    # Linear ramp down from delta_max to delta_high.
    frac = (q_dynamic - qbar_full) / (qbar_high - qbar_full)
    return delta_max + frac * (delta_high - delta_max)


def fin_controller(t, state, controller, config, reference, environment):
    """
    Callback for RocketPy GenericSurface integration.

    This function is idempotent per effective timestamp: repeated calls with
    the same ``t`` return the previously computed command without advancing
    controller state a second time.

    Parameters
    ----------
    t : float
        Simulation time (s).
    state : list
        RocketPy state vector: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz] (ENU).
    controller : dict
        Mutable controller state dictionary.
    config : Config
        Execution configuration.
    reference : dict
        Loaded reference trajectory (see ``src.reference``).
    environment : rocketpy.Environment
        Atmospheric environment for density and wind queries.

    Returns
    -------
    numpy.ndarray
        Fin deflection commands [delta1, delta2, delta3, delta4] in radians.
    """
    # ------------------------------------------------------------------
    # 0. Idempotency guard: detect duplicate callback at same timestamp
    # ------------------------------------------------------------------
    last_t = controller.get("_last_callback_t", None)
    if last_t is not None and abs(float(t) - float(last_t)) < _TIME_TOL:
        # Duplicate callback: return existing command, increment counter
        controller["_duplicate_callback_count"] = (
            controller.get("_duplicate_callback_count", 0) + 1
        )
        diag = controller.setdefault("_diagnostics", [])
        diag.append({
            "time_s": float(t),
            "raw_deltas_rad": controller.get("_last_raw_deltas", np.zeros(4)).tolist(),
            "limited_deltas_rad": controller["current_deltas"].tolist(),
            "control_active": False,
            "cutoff_reason": "duplicate_callback",
            "q_dynamic_pa": float(controller.get("last_q_dynamic", 0.0)),
            "airspeed_m_s": float(controller.get("last_airspeed", 0.0)),
            "position_error_enu_m": [0.0, 0.0, 0.0],
            "velocity_error_enu_m_s": [0.0, 0.0, 0.0],
            "attitude_error_quat": [1.0, 0.0, 0.0, 0.0],
            "commanded_accel_enu_m_s2": [0.0, 0.0, 0.0],
            "delta_limit_rad": float(controller.get("_last_delta_limit", controller.get("delta_max_rad", 0.349))),
            "effective_cD": 0.0,
        })
        return controller["current_deltas"].copy()

    controller["_last_callback_t"] = float(t)

    # ------------------------------------------------------------------
    # 1. State extraction
    # ------------------------------------------------------------------
    pos = np.array(state[0:3])
    vel = np.array(state[3:6])
    q_real = np.array(state[6:10])   # [w, x, y, z] ENU -> Body
    w_body = np.array(state[10:13])

    # Local altitude above launch pad
    z_asl = pos[2]
    z_local = z_asl - config.elevation_asl_m
    vz = vel[2]

    # ------------------------------------------------------------------
    # 2. Sample reference trajectory
    # ------------------------------------------------------------------
    from src.reference import sample_reference
    ref_sample = sample_reference(reference, t)
    pos_ref = ref_sample['position_enu_m']   # Local ENU
    vel_ref = ref_sample['velocity_enu_m_s']

    ref_time_limit = reference['time_s'][-1]

    # ------------------------------------------------------------------
    # 3. Atmospheric data (density, wind) from RocketPy Environment
    # ------------------------------------------------------------------
    try:
        rho = float(environment.density(z_asl))       # kg/m³, at ASL height
    except Exception:
        rho = 1.225  # ISA sea-level fallback

    try:
        wind_e = float(environment.wind_velocity_x(z_asl))  # East component (m/s)
        wind_n = float(environment.wind_velocity_y(z_asl))   # North component (m/s)
    except Exception:
        wind_e = 0.0
        wind_n = 0.0

    # Wind vector in ENU (assume no vertical wind component)
    wind_enu = np.array([wind_e, wind_n, 0.0])

    # Airspeed vector and magnitude (inertial - wind)
    vel_air = vel - wind_enu
    airspeed = np.linalg.norm(vel_air)

    # Dynamic pressure: q = 0.5 * rho * V_air^2
    q_dynamic = 0.5 * rho * airspeed ** 2

    # Store diagnostics in controller state (always, even when control is off)
    controller["last_rho"] = rho
    controller["last_wind_enu"] = wind_enu
    controller["last_airspeed"] = airspeed
    controller["last_q_dynamic"] = q_dynamic

    # ------------------------------------------------------------------
    # 4. Control activation / cutoff logic
    # ------------------------------------------------------------------
    # Conditions to DISABLE control:
    #   - Before control start delay
    #   - Below minimum activation height
    #   - After apogee (vz <= 0, rocket descending)
    #   - Beyond reference time horizon
    #   - Dynamic pressure below minimum (loss of aerodynamic authority)
    q_min_cutoff = getattr(config, 'q_min_cutoff_pa', 100.0)

    cutoff_reason = None
    if t < config.control_start_delay_s:
        cutoff_reason = "before_delay"
    elif z_local < config.control_start_min_height_above_launch_m:
        cutoff_reason = "below_min_height"
    elif vz <= 0:
        cutoff_reason = "descending"
    elif t > ref_time_limit:
        cutoff_reason = "beyond_ref_horizon"
    elif q_dynamic < q_min_cutoff:
        cutoff_reason = "low_q_dynamic"

    if cutoff_reason is not None:
        deltas = np.zeros(4)
        controller["deltas_history"][float(t)] = deltas
        controller["current_deltas"] = deltas
        # Reset integral on cutoff to prevent stale accumulation
        controller["integral_error"] = np.zeros(2)
        controller["previous_error"] = np.zeros(3)

        # Record diagnostic for cutoff sample
        diag = controller.setdefault("_diagnostics", [])
        pos_local_cut = pos - np.array([0, 0, config.elevation_asl_m])
        e_pos_cut = pos_ref - pos_local_cut
        e_vel_cut = vel_ref - vel
        diag.append({
            "time_s": float(t),
            "raw_deltas_rad": [0.0, 0.0, 0.0, 0.0],
            "limited_deltas_rad": [0.0, 0.0, 0.0, 0.0],
            "control_active": False,
            "cutoff_reason": cutoff_reason,
            "q_dynamic_pa": float(q_dynamic),
            "airspeed_m_s": float(airspeed),
            "position_error_enu_m": e_pos_cut.tolist(),
            "velocity_error_enu_m_s": e_vel_cut.tolist(),
            "attitude_error_quat": [1.0, 0.0, 0.0, 0.0],
            "commanded_accel_enu_m_s2": [0.0, 0.0, 0.0],
            "delta_limit_rad": 0.0,
            "effective_cD": 0.0,
        })
        return deltas

    # ------------------------------------------------------------------
    # 5. Normalize position to local ENU for tracking
    # ------------------------------------------------------------------
    pos_local = pos - np.array([0, 0, config.elevation_asl_m])

    # ------------------------------------------------------------------
    # 6. Outer-loop guidance: PD + feedforward + gravity + wind compensation
    # ------------------------------------------------------------------
    # Reference acceleration (numerical derivative of reference velocity)
    from src.reference import compute_reference_acceleration
    a_ref = compute_reference_acceleration(reference, t)

    # Position and velocity errors (inertial, ENU)
    e_pos = pos_ref - pos_local
    e_vel = vel_ref - vel   # Pure inertial derivative error (NOT airspeed)

    # Wind compensation: feedforward disturbance rejection.
    # Proportional on local wind velocity to command attitude into the wind.
    K_wind = getattr(config, 'K_wind_comp', 0.3)
    a_wind_comp = K_wind * wind_enu

    # Gravity compensation: +[0, 0, g] in ENU to counteract downward gravity.
    # This biases the commanded acceleration upward so the rocket generates
    # sufficient lift to maintain the trajectory against gravitational pull.
    gravity_vec = np.array([0, 0, abs(environment.gravity(z_asl))])

    # Full commanded acceleration in ENU
    accel_cmd_enu = (a_ref
                     + config.Kp_guidance * e_pos
                     + config.Kd_guidance * e_vel
                     + gravity_vec
                     + a_wind_comp)

    # Guidance command magnitude limiter.
    # The fins can only tilt the rocket a few degrees; beyond that, additional
    # commanded acceleration is unachievable and only drives saturation.
    # Limit the correction component (excluding gravity) to a_max_correction.
    a_max_correction = getattr(config, 'a_max_guidance_correction_m_s2', 15.0)
    correction_cmd = accel_cmd_enu - gravity_vec
    correction_mag = np.linalg.norm(correction_cmd)
    if correction_mag > a_max_correction > 0:
        correction_cmd = correction_cmd * (a_max_correction / correction_mag)
        accel_cmd_enu = correction_cmd + gravity_vec

    # ------------------------------------------------------------------
    # 7. Desired attitude (ENU -> Body)
    # ------------------------------------------------------------------
    q_ref = compute_desired_attitude(accel_cmd_enu, config)

    # ------------------------------------------------------------------
    # 8. Attitude PID (Body Frame) with anti-windup
    # ------------------------------------------------------------------
    # Error quaternion: q_error = q_ref * conjugate(q_real)
    q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))

    # Error mapping: RocketPy uses Body Z as longitudinal (Roll), and X/Y as transverse (Pitch/Yaw).
    #   - Roll: Axis Z (index 2 / qz, w_body[2])
    #   - Pitch: Axis X (index 0 / qx, w_body[0])
    #   - Yaw: Axis Y (index 1 / qy, w_body[1])
    error_vec = q_error[1:4]  # [qx, qy, qz]
    e_pitch = error_vec[0]
    e_yaw = error_vec[1]
    e_roll = error_vec[2]

    dt = config.control_dt_s

    # --- Avoid Derivative Spike on First Step ---
    if np.all(controller["previous_error"] == 0.0):
        controller["previous_error"] = error_vec.copy()

    # --- Anti-windup: per-axis back-calculation (Åström & Hägglund, 1995) ---
    # Roll uses PD-only (no integral action), so we only track pitch/yaw
    # integral states.  Saturation is detected per-axis to prevent freezing
    # integral on an unsaturated axis due to saturation on another.
    #
    # Mixer mapping (4 fins in cross configuration):
    #   Fin 1 (0°):   u_yaw + u_roll
    #   Fin 2 (90°):  u_pitch + u_roll
    #   Fin 3 (180°): -u_yaw + u_roll
    #   Fin 4 (270°): -u_pitch + u_roll
    # So pitch saturates via fins 2/4, yaw via fins 1/3.

    # Compute raw (unsaturated) virtual control outputs to detect saturation
    raw_u_roll = (config.Kp_roll * e_roll
                  - config.Kd_attitude * w_body[2])
    raw_u_pitch = (config.Kp_attitude * e_pitch
                   + config.Ki_attitude * (controller["integral_error"][0] + e_pitch * dt)
                   + config.Kd_attitude * w_body[0])
    raw_u_yaw = (config.Kp_attitude * e_yaw
                 + config.Ki_attitude * (controller["integral_error"][1] + e_yaw * dt)
                 + config.Kd_attitude * w_body[1])

    raw_deltas = np.array([
        raw_u_yaw + raw_u_roll,
        raw_u_pitch + raw_u_roll,
       -raw_u_yaw + raw_u_roll,
       -raw_u_pitch + raw_u_roll
    ])

    delta_max = controller["delta_max_rad"]
    sat_mask = np.abs(raw_deltas) > delta_max

    # Per-axis saturation: pitch saturates via fins 2/4 (indices 1,3),
    # yaw saturates via fins 1/3 (indices 0,2).
    pitch_saturated = sat_mask[1] or sat_mask[3]
    yaw_saturated = sat_mask[0] or sat_mask[2]

    # Update pitch/yaw integral independently (conditional integration).
    # Roll has no integral term (Ki_roll = 0), so no roll integral is stored.
    integral = controller["integral_error"]
    if not pitch_saturated:
        integral[0] += e_pitch * dt
    if not yaw_saturated:
        integral[1] += e_yaw * dt

    # Use measured body-rate damping instead of differentiating quaternion error.
    # Error differencing is numerically noisy at the controller callback cadence
    # and was exciting high-frequency fin chatter during the high-q ascent.
    controller["previous_error"] = error_vec.copy()

    # Roll damper (PD-only, no integral)
    u_roll = config.Kp_roll * e_roll - config.Kd_attitude * w_body[2]

    # Pitch/Yaw control outputs (PID with per-axis anti-windup)
    u_pitch = (config.Kp_attitude * e_pitch +
               config.Ki_attitude * integral[0] +
               config.Kd_attitude * w_body[0])

    u_yaw = (config.Kp_attitude * e_yaw +
             config.Ki_attitude * integral[1] +
             config.Kd_attitude * w_body[1])

    # ------------------------------------------------------------------
    # 9. Mixer: virtual control to 4 rear fins (cross configuration)
    # ------------------------------------------------------------------
    # Fin 1 (0°, right):  u_yaw + u_roll
    # Fin 2 (90°, top):   u_pitch + u_roll
    # Fin 3 (180°, left): -u_yaw + u_roll
    # Fin 4 (270°, bottom): -u_pitch + u_roll
    deltas = np.array([
        u_yaw + u_roll,
        u_pitch + u_roll,
       -u_yaw + u_roll,
       -u_pitch + u_roll
    ])

    # ------------------------------------------------------------------
    # 10. Actuator limits (rate and position, with q-bar authority scheduling)
    # ------------------------------------------------------------------
    try:
        delta_dot_max = controller["delta_dot_max_rad_s"]
    except KeyError as e:
        raise RuntimeError(
            f"Controller state missing required limit: {e}. "
            "Ensure rocket TOML defines control_actuation limits."
        ) from e

    # q-bar scheduled authority limit
    delta_limit_rad = _compute_qbar_authority_limit(q_dynamic, config)

    prev_deltas = controller.get("current_deltas", np.zeros(4))
    command_filter_tau = getattr(config, 'actuator_command_filter_tau_s', 0.0)
    if command_filter_tau > 0.0:
        alpha = dt / (command_filter_tau + dt)
        deltas = prev_deltas + alpha * (deltas - prev_deltas)

    max_step = delta_dot_max * dt
    deltas = np.clip(deltas, prev_deltas - max_step, prev_deltas + max_step)
    deltas = np.clip(deltas, -delta_limit_rad, delta_limit_rad)

    # Store for raw_deltas diagnostic (before clipping to delta_max)
    controller["_last_raw_deltas"] = raw_deltas.copy()
    controller["_last_delta_limit"] = float(delta_limit_rad)

    # ------------------------------------------------------------------
    # 11. Diagnostic tracking
    # ------------------------------------------------------------------
    # Track saturation time based on ACTUAL (clamped) deltas, not raw.
    if delta_limit_rad > 1e-9 and np.any(np.abs(deltas) >= 0.95 * delta_limit_rad):
        controller["saturation_time_s"] = controller.get("saturation_time_s", 0.0) + dt

    # Compute effective control cD for diagnostics
    cN_delta = getattr(config, '_cN_delta_per_rad', 9.343586365106)
    cy_delta = getattr(config, '_cy_delta_per_rad', 9.343586365106)
    k_drag = getattr(config, '_k_drag_induced', 0.295907824866)
    delta_pitch = (deltas[1] - deltas[3]) / 2.0
    delta_yaw = (deltas[0] - deltas[2]) / 2.0
    cL = cN_delta * delta_pitch
    cQ = cy_delta * delta_yaw
    effective_cD = k_drag * (cL**2 + cQ**2)

    # Update current state for GenericSurface adapter
    controller["current_deltas"] = deltas

    # Store history
    controller["deltas_history"][float(t)] = deltas

    # Record comprehensive diagnostic
    diag = controller.setdefault("_diagnostics", [])
    diag.append({
        "time_s": float(t),
        "raw_deltas_rad": raw_deltas.tolist(),
        "limited_deltas_rad": deltas.tolist(),
        "control_active": True,
        "cutoff_reason": "",
        "q_dynamic_pa": float(q_dynamic),
        "airspeed_m_s": float(airspeed),
        "position_error_enu_m": e_pos.tolist(),
        "velocity_error_enu_m_s": e_vel.tolist(),
        "attitude_error_quat": q_error.tolist(),
        "commanded_accel_enu_m_s2": accel_cmd_enu.tolist(),
        "delta_limit_rad": float(delta_limit_rad),
        "effective_cD": float(effective_cD),
    })

    return deltas


def build_controller(config):
    """
    Initializes the controller state dictionary.

    Parameters
    ----------
    config : Config
        Execution configuration.

    Returns
    -------
    dict
        Mutable controller state with pre-allocated fields, including
        diagnostics for auditing controller behavior after a run.

    Notes
    -----
    ``integral_error`` is a 2-element vector ``[pitch, yaw]``.
    Roll uses PD-only control (no integral action), so no roll integral
    state is stored — this prevents unused roll integral from accumulating
    and leaking into pitch/yaw when a shared anti-windup gate is used.
    """
    return {
        "integral_error": np.zeros(2),      # [pitch, yaw] only (no roll Ki)
        "previous_error": np.zeros(3),      # [roll, pitch, yaw]
        "current_deltas": np.zeros(4),
        "deltas_history": {},               # time -> [d1, d2, d3, d4]
        "saturation_time_s": 0.0,           # cumulative time at saturation
        "last_rho": 1.225,                  # diagnostic: last density (kg/m³)
        "last_wind_enu": np.zeros(3),       # diagnostic: last wind vector (m/s)
        "last_airspeed": 0.0,               # diagnostic: last airspeed (m/s)
        "last_q_dynamic": 0.0,              # diagnostic: last dynamic pressure (Pa)
        # --- Idempotency and diagnostics (Task 1 & 2) ---
        "_last_callback_t": None,           # last effective callback timestamp
        "_duplicate_callback_count": 0,     # count of duplicate callback detections
        "_diagnostics": [],                 # list of per-sample diagnostic dicts
        "_last_raw_deltas": np.zeros(4),    # raw deltas before rate/authority limit
        "_last_delta_limit": 0.0,           # effective delta_limit_rad at last sample
    }


def compute_desired_attitude(a_cmd_enu, config):
    """
    Computes the desired attitude quaternion (ENU -> Body) that aligns the
    rocket nose (Body +Z) with the commanded acceleration vector.

    Parameters
    ----------
    a_cmd_enu : numpy.ndarray
        Commanded acceleration vector in local ENU (m/s²).
    config : Config
        Execution configuration (unused, reserved for future extensions).

    Returns
    -------
    numpy.ndarray
        Unit quaternion [w, x, y, z] representing ENU -> Body rotation.
    """
    norm = np.linalg.norm(a_cmd_enu)
    if norm < 1e-6:
        direction = np.array([0, 0, 1])  # Default to vertical
    else:
        direction = a_cmd_enu / norm

    # q_ref is ENU -> Body: rotates 'direction' to Body [0, 0, 1].
    q_ref = utils.quaternion_from_vectors(direction, np.array([0, 0, 1]))
    return q_ref
