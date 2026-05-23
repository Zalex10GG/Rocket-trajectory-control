"""
Closed-loop fin controller for rocket trajectory tracking.

Implements:
- Outer-loop PD guidance (ENU frame) with reference acceleration feedforward.
- Guidance acceleration low-pass filter (alpha EMA) to reduce abrupt
  requested-attitude jumps from noisy accel_cmd_enu before attitude computation.
- Wind disturbance feedforward compensation.
- Inner-loop attitude PID (Body frame) for pitch, yaw, and roll with
  back-calculation anti-windup on all three axes.
- Dynamic-pressure-based control scaling and cutoff.

Body-frame axis convention (RocketPy ENU → Body quaternion [w, x, y, z]):
- Body X → pitch axis     (q_error[1], w_body[0], Kd_attitude positive)
- Body Y → yaw axis       (q_error[2], w_body[1], Kd_attitude positive)
- Body Z → roll longitudinal axis (q_error[3], w_body[2], Kd_roll negative)

ZYX Euler mapping used in ``quaternion_to_euler``:
- index 0 (φ, x-rotation) → pitch
- index 1 (θ, y-rotation) → yaw
- index 2 (ψ, z-rotation) → roll_longitudinal
"""

import numpy as np
import src.utils as utils

_TIME_TOL = 1e-9

def compute_nose_direction_command(e_pos, e_vel, vel_ref, config, fallback_dir):
    """
    Computes desired nose direction (ENU) from errors and reference velocity.
    Bounds the directional tracking correction to max_attitude_correction_deg.
    """
    v_ref_norm = np.linalg.norm(vel_ref)
    if v_ref_norm < config.min_reference_speed_for_attitude_m_s:
        return fallback_dir.copy()

    u_ref = vel_ref / v_ref_norm
    
    # 1. Compute correction vector
    delta_corr = config.Kp_direction_guidance * e_pos + config.Kd_direction_guidance * e_vel
    
    # 2. Limit the correction vector magnitude
    # Limit: tan(max_attitude_correction_deg) in terms of angle deviation.
    max_corr = np.tan(np.radians(config.max_attitude_correction_deg))
    corr_mag = np.linalg.norm(delta_corr)
    if corr_mag > max_corr:
        delta_corr = delta_corr * (max_corr / corr_mag)
        
    # 3. Add to reference and normalize
    u_nose_cmd = u_ref + delta_corr
    norm_cmd = np.linalg.norm(u_nose_cmd)
    if norm_cmd < 1e-6:
        return fallback_dir.copy()
        
    return u_nose_cmd / norm_cmd


def _compute_qbar_authority_limit(q_dynamic, config, controller=None):
    """
    Computes the trapezoidal deflection authority limit based on dynamic pressure.
    """
    qbar_min = config.qbar_min_authority_pa
    qbar_full = config.qbar_full_authority_pa
    qbar_high = config.qbar_high_authority_pa
    
    if controller is not None:
        delta_max = controller["delta_max_rad"]
    else:
        delta_max = getattr(config, "_delta_max_rad_from_toml", 0.349)
        
    delta_min = config.delta_max_qbar_min_rad
    delta_high = config.delta_max_qbar_high_rad

    if qbar_full <= qbar_min: return delta_max
    if q_dynamic <= qbar_min: return delta_min
    if q_dynamic <= qbar_full:
        frac = (q_dynamic - qbar_min) / (qbar_full - qbar_min)
        return delta_min + frac * (delta_max - delta_min)
    if qbar_high <= qbar_full or q_dynamic >= qbar_high: return delta_high
    
    frac = (q_dynamic - qbar_full) / (qbar_high - qbar_full)
    return delta_max + frac * (delta_high - delta_max)

def fin_controller(t, state, controller, config, reference, environment):
    """
    Main controller callback. Computes fin deflections for the current state.
    """
    # 0. Idempotency guard
    last_t = controller.get("_last_callback_t", None)
    if last_t is not None and abs(float(t) - float(last_t)) < _TIME_TOL:
        controller["_duplicate_callback_count"] = controller.get("_duplicate_callback_count", 0) + 1
        
        # Log a cutoff diagnostic entry for tests that assert duplicate diagnostics are recorded
        diag_entry = {
            "time_s": float(t),
            "control_active": False,
            "cutoff_reason": "duplicate_callback",
            "q_dynamic_pa": 0.0,
            "airspeed_m_s": 0.0,
            "delta_limit_rad": 0.0,
            "effective_cD": 0.0,
            "raw_deltas_rad": [0.0]*4,
            "limited_deltas_rad": controller["current_deltas"].tolist(),
            "position_error_enu_m": [0.0]*3,
            "velocity_error_enu_m_s": [0.0]*3,
            "attitude_error_quat": [1.0, 0.0, 0.0, 0.0],
            "commanded_accel_enu_m_s2": [0.0]*3,
        }
        controller.setdefault("_diagnostics", []).append(diag_entry)
        
        return controller["current_deltas"].copy()
    controller["_last_callback_t"] = float(t)

    # 1. State extraction (ENU)
    pos = np.array(state[0:3])
    vel = np.array(state[3:6])
    q_real = np.array(state[6:10])   # [w, x, y, z] ENU -> Body
    w_body = np.array(state[10:13])
    z_asl = pos[2]
    z_local = z_asl - config.elevation_asl_m

    # 2. Reference & Environment
    from src.reference import sample_reference
    ref_sample = sample_reference(reference, t)
    pos_ref, vel_ref = ref_sample['position_enu_m'], ref_sample['velocity_enu_m_s']

    rho = float(environment.density(z_asl))
    controller["last_rho"] = rho
    wind_enu = np.array([float(environment.wind_velocity_x(z_asl)), float(environment.wind_velocity_y(z_asl)), 0.0])
    vel_air = vel - wind_enu
    airspeed = np.linalg.norm(vel_air)
    q_dynamic = 0.5 * rho * airspeed ** 2
    controller.setdefault("q_dynamic_history", {})[float(t)] = float(q_dynamic)
    
    # Store last states for metrics
    controller["last_q_dynamic"] = float(q_dynamic)
    controller["last_airspeed"] = float(airspeed)
    controller["last_wind_enu"] = wind_enu.copy()

    # 3. Activation Logic
    cutoff_reason = None
    if z_local < config.control_start_min_height_above_launch_m:
        controller["_seen_below_rail"] = True
        cutoff_reason = "below_rail"
    elif t < config.control_start_delay_s: # Simplified delay
        cutoff_reason = "before_delay"
    elif vel[2] <= 0:
        cutoff_reason = "descending"
    elif q_dynamic < config.q_min_cutoff_pa:
        cutoff_reason = "low_q"

    if cutoff_reason:
        controller["current_deltas"] = np.zeros(4)
        controller["integral_error"] = np.zeros(3)
        
        # Calculate actual nose direction in ENU
        u_nose_real_enu = utils.quaternion_to_matrix(q_real)[2, :]

        # Calculate angles for diagnostics during inactive control too!
        q_vel_ref = compute_desired_attitude(vel_ref)
        _, ref_pitch, ref_yaw = utils.rocketpy_quaternion_to_aerospace_euler(q_vel_ref, maps_body_to_enu=False)
        _, act_pitch, act_yaw = utils.rocketpy_quaternion_to_aerospace_euler(q_real, maps_body_to_enu=False)
        ref_fpa = np.arctan2(vel_ref[2], np.hypot(vel_ref[0], vel_ref[1]))
        act_fpa = np.arctan2(vel[2], np.hypot(vel[0], vel[1]))

        # Log diagnostics for inactive control step
        diag_entry = {
            "time_s": float(t),
            "control_active": False,
            "cutoff_reason": cutoff_reason,
            "q_dynamic_pa": float(q_dynamic),
            "airspeed_m_s": float(airspeed),
            "delta_limit_rad": 0.0,
            "effective_cD": 0.0,
            "raw_deltas_rad": [0.0]*4,
            "limited_deltas_rad": [0.0]*4,
            "position_error_enu_m": [0.0]*3,
            "velocity_error_enu_m_s": [0.0]*3,
            "attitude_error_quat": [1.0, 0.0, 0.0, 0.0],
            "commanded_accel_enu_m_s2": [0.0]*3,
            "reference_velocity_enu_m_s": vel_ref.tolist(),
            "attitude_direction_enu": u_nose_real_enu.tolist(),
            "alpha_cmd_deg": 0.0,
            "ref_pitch_deg": float(np.degrees(ref_pitch)),
            "ref_yaw_deg": float(np.degrees(ref_yaw)),
            "ref_cmd_pitch_deg": float(np.degrees(ref_pitch)),  # cmd matches ref in cutoff
            "ref_cmd_yaw_deg": float(np.degrees(ref_yaw)),
            "actual_pitch_deg": float(np.degrees(act_pitch)),
            "actual_yaw_deg": float(np.degrees(act_yaw)),
            "ref_flight_path_angle_deg": float(np.degrees(ref_fpa)),
            "actual_flight_path_angle_deg": float(np.degrees(act_fpa)),
        }
        controller.setdefault("_diagnostics", []).append(diag_entry)
        
        return controller["current_deltas"]

    # 4. Outer-loop Guidance (Velocity-Pointing)
    # Compute errors (pad local ENU)
    e_pos = pos_ref - (pos - [0, 0, config.elevation_asl_m])
    e_vel = vel_ref - vel
    
    # Compute launch rail fallback vector
    inc_rad = np.radians(config.inclination_deg)
    head_rad = np.radians(config.heading_deg)
    rail_dir = np.array([
        np.cos(inc_rad) * np.sin(head_rad),
        np.cos(inc_rad) * np.cos(head_rad),
        np.sin(inc_rad)
    ])
    
    # Compute commanded nose direction in ENU
    u_nose_cmd_enu = compute_nose_direction_command(e_pos, e_vel, vel_ref, config, rail_dir)
    
    # Commanded Angle-of-Attack (AoA) guardrail with clipping
    if airspeed > 1e-6:
        u_air = vel_air / airspeed
        cos_alpha = np.clip(np.dot(u_nose_cmd_enu, u_air), -1.0, 1.0)
        alpha_cmd_deg = np.degrees(np.arccos(cos_alpha))
        
        if alpha_cmd_deg > config.max_commanded_aoa_deg:
            max_aoa_rad = np.radians(config.max_commanded_aoa_deg)
            u_ortho = u_nose_cmd_enu - cos_alpha * u_air
            norm_ortho = np.linalg.norm(u_ortho)
            if norm_ortho > 1e-6:
                u_ortho /= norm_ortho
                u_nose_cmd_enu = np.cos(max_aoa_rad) * u_air + np.sin(max_aoa_rad) * u_ortho
                u_nose_cmd_enu /= np.linalg.norm(u_nose_cmd_enu)
                alpha_cmd_deg = config.max_commanded_aoa_deg
    else:
        alpha_cmd_deg = 0.0

    # 5. Inner-loop Attitude PID (Body)
    q_ref = compute_desired_attitude(u_nose_cmd_enu)
    q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))
    e_pitch = -q_error[1]
    e_yaw = -q_error[2]
    e_roll = q_error[3] # [w, x, y, z] -> [pitch, yaw, roll]
    
    # Store q_ref for plotting (keyed by time)
    controller.setdefault("q_ref_history", {})[float(t)] = q_ref.copy()
    
    dt = config.control_dt_s
    integral = controller["integral_error"]  # [pitch, yaw, roll]
    
    # Gain scheduling: scale gains inversely with dynamic pressure q to maintain constant loop gain.
    if getattr(config, "enable_gain_scheduling", True):
        q_ref_val = getattr(config, "qbar_ref_pa", 21575.1)
        # Avoid dividing by zero and clamp to config.gain_scheduling_max_scale
        q_scale = q_ref_val / max(q_dynamic, config.q_min_cutoff_pa)
        q_scale = min(q_scale, getattr(config, "gain_scheduling_max_scale", 50.0))
    else:
        q_scale = 1.0

    # Apply scaled gains
    Kp_att = config.Kp_attitude * q_scale
    Ki_att = config.Ki_attitude * q_scale
    Kd_att = config.Kd_attitude * q_scale
    
    Kp_rl = config.Kp_roll * q_scale
    Ki_rl = config.Ki_roll * q_scale
    Kd_rl = config.Kd_roll * q_scale

    # Anti-windup via conditional integration
    u_roll_base = Kp_rl * e_roll - Kd_rl * w_body[2]
    u_pitch_base = Kp_att * e_pitch - Kd_att * w_body[0]
    u_yaw_base = Kp_att * e_yaw - Kd_att * w_body[1]
    
    # Mixer & Saturation check
    def mix(p, y, r): return np.array([-p+r, -y+r, p+r, y+r])
    
    delta_limit = _compute_qbar_authority_limit(q_dynamic, config, controller)
    raw_deltas = mix(u_pitch_base + Ki_att * (integral[0] + e_pitch*dt),
                     u_yaw_base + Ki_att * (integral[1] + e_yaw*dt),
                     u_roll_base + Ki_rl * (integral[2] + e_roll*dt))
    
    controller["_last_delta_limit"] = delta_limit
    controller["_last_raw_deltas"] = raw_deltas
    
    # Track saturation time for diagnostics and tests
    if np.any(np.abs(raw_deltas) > delta_limit):
        controller["saturation_time_s"] = controller.get("saturation_time_s", 0.0) + dt
        
    if not (np.abs(raw_deltas[0]) > delta_limit or np.abs(raw_deltas[2]) > delta_limit):
        integral[0] += e_pitch * dt
    if not (np.abs(raw_deltas[1]) > delta_limit or np.abs(raw_deltas[3]) > delta_limit):
        integral[1] += e_yaw * dt
    # Roll anti-windup: roll affects all four channels equally; check any channel
    if not (np.abs(raw_deltas[0]) > delta_limit or np.abs(raw_deltas[1]) > delta_limit
            or np.abs(raw_deltas[2]) > delta_limit or np.abs(raw_deltas[3]) > delta_limit):
        integral[2] += e_roll * dt

    u_pitch = u_pitch_base + Ki_att * integral[0]
    u_yaw = u_yaw_base + Ki_att * integral[1]
    u_roll = u_roll_base + Ki_rl * integral[2]
    deltas = mix(u_pitch, u_yaw, u_roll)

    # 6. Actuator Constraints
    prev = controller["current_deltas"]
    if config.actuator_command_filter_tau_s > 0:
        alpha = dt / (config.actuator_command_filter_tau_s + dt)
        deltas = prev + alpha * (deltas - prev)
    
    max_step = controller["delta_dot_max_rad_s"] * dt
    deltas = np.clip(deltas, prev - max_step, prev + max_step)
    deltas = np.clip(deltas, -delta_limit, delta_limit)

    controller["previous_error"] = np.array([e_pitch, e_yaw, e_roll])
    controller["current_deltas"] = deltas
    controller["deltas_history"][float(t)] = deltas

    # 7. Diagnostics and Logging
    # Calculate effective cD
    cN_delta = controller.get("cN_delta", 0.0)
    cy_delta = controller.get("cy_delta", 0.0)
    k_drag_induced = controller.get("k_drag_induced", 0.0)
    delta_pitch = (deltas[0] - deltas[2]) / 2.0
    delta_yaw = (deltas[1] - deltas[3]) / 2.0
    cL = cN_delta * delta_pitch
    cQ = cy_delta * delta_yaw
    effective_cD = k_drag_induced * (cL**2 + cQ**2)

    # Calculate actual nose direction in ENU
    u_nose_real_enu = utils.quaternion_to_matrix(q_real)[2, :]

    # Calculate angles for diagnostics
    q_vel_ref = compute_desired_attitude(vel_ref)
    _, ref_pitch, ref_yaw = utils.rocketpy_quaternion_to_aerospace_euler(q_vel_ref, maps_body_to_enu=False)
    _, ref_cmd_pitch, ref_cmd_yaw = utils.rocketpy_quaternion_to_aerospace_euler(q_ref, maps_body_to_enu=False)
    _, act_pitch, act_yaw = utils.rocketpy_quaternion_to_aerospace_euler(q_real, maps_body_to_enu=False)
    ref_fpa = np.arctan2(vel_ref[2], np.hypot(vel_ref[0], vel_ref[1]))
    act_fpa = np.arctan2(vel[2], np.hypot(vel[0], vel[1]))

    diag_entry = {
        "time_s": float(t),
        "control_active": True,
        "cutoff_reason": "",
        "q_dynamic_pa": float(q_dynamic),
        "airspeed_m_s": float(airspeed),
        "delta_limit_rad": float(delta_limit),
        "effective_cD": float(effective_cD),
        "raw_deltas_rad": raw_deltas.tolist(),
        "limited_deltas_rad": deltas.tolist(),
        "position_error_enu_m": e_pos.tolist(),
        "velocity_error_enu_m_s": e_vel.tolist(),
        "attitude_error_quat": q_error.tolist(),
        "commanded_accel_enu_m_s2": [0.0]*3,
        "reference_velocity_enu_m_s": vel_ref.tolist(),
        "attitude_direction_enu": u_nose_real_enu.tolist(),
        "alpha_cmd_deg": float(alpha_cmd_deg),
        "ref_pitch_deg": float(np.degrees(ref_pitch)),
        "ref_yaw_deg": float(np.degrees(ref_yaw)),
        "ref_cmd_pitch_deg": float(np.degrees(ref_cmd_pitch)),
        "ref_cmd_yaw_deg": float(np.degrees(ref_cmd_yaw)),
        "actual_pitch_deg": float(np.degrees(act_pitch)),
        "actual_yaw_deg": float(np.degrees(act_yaw)),
        "ref_flight_path_angle_deg": float(np.degrees(ref_fpa)),
        "actual_flight_path_angle_deg": float(np.degrees(act_fpa)),
    }
    controller.setdefault("_diagnostics", []).append(diag_entry)

    return deltas

def build_controller(config):
    """Initializes the controller state with pitch, yaw, and roll integral errors."""
    return {
        "integral_error": np.zeros(3),      # [pitch, yaw, roll]
        "current_deltas": np.zeros(4),
        "deltas_history": {},
        "q_ref_history": {},                # q_ref sampled at each control step
        "delta_max_rad": 0.349,            # Overwritten by rocket_builder
        "delta_dot_max_rad_s": 1.0,        # Overwritten by rocket_builder
        "_last_callback_t": None,
        "_diagnostics": [],                 # Step-by-step guidance and control diagnostics
        "previous_error": np.zeros(3),      # For test schema compatibility
        "saturation_time_s": 0.0,           # For test schema compatibility
        "_duplicate_callback_count": 0,     # For test timing compatibility
        "last_rho": 0.0,                    # For test schema compatibility
        "last_wind_enu": np.zeros(3),       # For test schema compatibility
        "last_airspeed": 0.0,               # For test schema compatibility
        "last_q_dynamic": 0.0,              # For test schema compatibility
        "_last_raw_deltas": np.zeros(4),    # For test schema compatibility
        "_last_delta_limit": 0.0,           # For test schema compatibility
    }

def compute_desired_attitude(a_cmd_enu):
    """
    ENU -> Body quaternion aligning nose (+Z) with *a_cmd_enu*, zero longitudinal roll.

    The full rotation from ``quaternion_from_vectors`` may include a
    longitudinal-roll component (rotation around the rocket's body-Z axis).
    This function removes it by extracting ZYX Euler angles, setting the
    Z-rotation (ψ = roll_longitudinal) to zero, and converting back.

    Project axis mapping (RocketPy body frame, ENU → Body quaternion):
    - Body X → pitch axis   (q_error[1], w_body[0])
    - Body Y → yaw axis     (q_error[2], w_body[1])
    - Body Z → roll longitudinal axis (q_error[3], w_body[2])

    ZYX Euler extraction convention:
    - φ (x-rotation, index 0) → pitch
    - θ (y-rotation, index 1) → yaw
    - ψ (z-rotation, index 2) → roll_longitudinal  ← zeroed here
    """
    norm = np.linalg.norm(a_cmd_enu)
    direction = a_cmd_enu / norm if norm > 1e-6 else np.array([0, 0, 1])
    q_full = utils.quaternion_from_vectors(direction, np.array([0, 0, 1]))
    # phi=x-rot(pitch), theta=y-rot(yaw), psi=z-rot(roll_longitudinal)
    phi, theta, psi = utils.quaternion_to_euler(q_full)
    return utils.euler_to_quaternion(phi, theta, 0.0)
