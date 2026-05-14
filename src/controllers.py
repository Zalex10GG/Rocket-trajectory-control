"""
Closed-loop fin controller for rocket trajectory tracking.

Implements:
- Outer-loop PD guidance (ENU frame) with reference acceleration feedforward.
- Wind disturbance feedforward compensation.
- Inner-loop attitude PID (Body frame) with back-calculation anti-windup.
- Dynamic-pressure-based control scaling and cutoff.
"""

import numpy as np
import src.utils as utils

_TIME_TOL = 1e-9

def _compute_qbar_authority_limit(q_dynamic, config, controller):
    """
    Computes the trapezoidal deflection authority limit based on dynamic pressure.
    """
    qbar_min = config.qbar_min_authority_pa
    qbar_full = config.qbar_full_authority_pa
    qbar_high = config.qbar_high_authority_pa
    delta_max = controller["delta_max_rad"]
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
    from src.reference import sample_reference, compute_reference_acceleration
    ref_sample = sample_reference(reference, t)
    pos_ref, vel_ref = ref_sample['position_enu_m'], ref_sample['velocity_enu_m_s']
    a_ref = compute_reference_acceleration(reference, t)

    rho = float(environment.density(z_asl))
    wind_enu = np.array([float(environment.wind_velocity_x(z_asl)), float(environment.wind_velocity_y(z_asl)), 0.0])
    vel_air = vel - wind_enu
    airspeed = np.linalg.norm(vel_air)
    q_dynamic = 0.5 * rho * airspeed ** 2

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
        controller["integral_error"] = np.zeros(2)
        return controller["current_deltas"]

    # 4. Outer-loop Guidance (ENU)
    e_pos, e_vel = pos_ref - (pos - [0,0,config.elevation_asl_m]), vel_ref - vel
    a_wind_comp = config.K_wind_comp * wind_enu
    gravity_vec = np.array([0, 0, abs(environment.gravity(z_asl))])
    
    accel_cmd_enu = (a_ref + config.Kp_guidance * e_pos + config.Kd_guidance * e_vel + gravity_vec + a_wind_comp)
    
    # Clip correction to avoid extreme attitudes
    corr = accel_cmd_enu - gravity_vec
    corr_mag = np.linalg.norm(corr)
    if corr_mag > config.a_max_guidance_correction_m_s2:
        accel_cmd_enu = corr * (config.a_max_guidance_correction_m_s2 / corr_mag) + gravity_vec

    # 5. Inner-loop Attitude PID (Body)
    q_ref = compute_desired_attitude(accel_cmd_enu)
    q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))
    e_pitch, e_yaw, e_roll = q_error[1], q_error[2], q_error[3] # [w, x, y, z] -> [pitch, yaw, roll]
    
    dt = config.control_dt_s
    integral = controller["integral_error"]
    
    # Anti-windup via conditional integration
    u_roll = config.Kp_roll * e_roll - config.Kd_attitude * w_body[2]
    u_pitch_base = config.Kp_attitude * e_pitch + config.Kd_attitude * w_body[0]
    u_yaw_base = config.Kp_attitude * e_yaw + config.Kd_attitude * w_body[1]
    
    # Mixer & Saturation check
    def mix(p, y, r): return np.array([p+r, y+r, -p+r, -y+r])
    
    delta_limit = _compute_qbar_authority_limit(q_dynamic, config, controller)
    raw_deltas = mix(u_pitch_base + config.Ki_attitude * (integral[0] + e_pitch*dt),
                     u_yaw_base + config.Ki_attitude * (integral[1] + e_yaw*dt), u_roll)
    
    if not (np.abs(raw_deltas[0]) > delta_limit or np.abs(raw_deltas[2]) > delta_limit):
        integral[0] += e_pitch * dt
    if not (np.abs(raw_deltas[1]) > delta_limit or np.abs(raw_deltas[3]) > delta_limit):
        integral[1] += e_yaw * dt

    u_pitch = u_pitch_base + config.Ki_attitude * integral[0]
    u_yaw = u_yaw_base + config.Ki_attitude * integral[1]
    deltas = mix(u_pitch, u_yaw, u_roll)

    # 6. Actuator Constraints
    prev = controller["current_deltas"]
    if config.actuator_command_filter_tau_s > 0:
        alpha = dt / (config.actuator_command_filter_tau_s + dt)
        deltas = prev + alpha * (deltas - prev)
    
    max_step = controller["delta_dot_max_rad_s"] * dt
    deltas = np.clip(deltas, prev - max_step, prev + max_step)
    deltas = np.clip(deltas, -delta_limit, delta_limit)

    controller["current_deltas"] = deltas
    controller["deltas_history"][float(t)] = deltas
    return deltas

def build_controller(config):
    """Initializes the controller state."""
    return {
        "integral_error": np.zeros(2),      # [pitch, yaw]
        "current_deltas": np.zeros(4),
        "deltas_history": {},
        "delta_max_rad": 0.349,            # Overwritten by rocket_builder
        "delta_dot_max_rad_s": 1.0,        # Overwritten by rocket_builder
        "_last_callback_t": None,
    }

def compute_desired_attitude(a_cmd_enu):
    """ENU -> Body quaternion aligning nose (+Z) with accel vector."""
    norm = np.linalg.norm(a_cmd_enu)
    direction = a_cmd_enu / norm if norm > 1e-6 else np.array([0, 0, 1])
    return utils.quaternion_from_vectors(direction, np.array([0, 0, 1]))
