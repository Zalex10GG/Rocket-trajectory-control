import numpy as np
import src.utils as utils

def fin_controller(t, state, controller, config, reference, gravity):
    """
    Callback for RocketPy GenericSurface integration.
    t: time (s)
    state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz] (ENU)
    gravity: local gravity magnitude (m/s^2) - Required, must be provided by simulation.
    """
    # 1. State extraction
    pos = np.array(state[0:3])
    vel = np.array(state[3:6])
    q_real = np.array(state[6:10]) # [w, x, y, z] ENU -> Body
    w_body = np.array(state[10:13])
    
    # 2. Control activation logic
    # Leon 2 burn time ~3.5s. Start control after burn and at safe height.
    # state[0:3] is ENU ASL from RocketPy internal integration
    z_asl = pos[2]
    # Use config.elevation_asl_m consistently
    z_local = z_asl - config.elevation_asl_m
    vz = vel[2]
    
    # 3. Guidance: Trajectory tracking (Local ENU)
    from src.reference import sample_reference
    ref_sample = sample_reference(reference, t)
    pos_ref = ref_sample['position_enu_m'] # Reference is already local ENU (0,0,0)
    vel_ref = ref_sample['velocity_enu_m_s']

    # Control cutoff logic: 
    # - If t is beyond reference time (assuming reference['time_s'][-1] is the limit)
    # - If vz <= 0 (reached apogee)
    # - If reference velocity is zero or negative (descending reference)
    ref_time_limit = reference['time_s'][-1]
    
    if (t < config.control_start_delay_s or 
        z_local < config.control_start_min_height_above_launch_m or 
        vz <= 0 or 
        t > ref_time_limit or 
        vel_ref[2] <= 0):
        
        deltas = np.zeros(4)
        controller["deltas_history"][float(t)] = deltas
        # Reset integral if needed, or just return
        return deltas

    # Normalize current position to local ENU for tracking
    pos_local = pos - np.array([0, 0, config.elevation_asl_m])
    
    # Simple PD guidance to get desired acceleration/direction in ENU
    # We want to align the velocity vector with the reference path
    # Desired acceleration is used to define the target pointing vector
    accel_cmd_enu = config.Kp_guidance * (pos_ref - pos_local) + config.Kd_guidance * (vel_ref - vel)
    # Add vertical component to bias the pointing vector upwards
    # This ensures the rocket maintains an upward orientation during tracking
    accel_cmd_enu += np.array([0, 0, gravity]) 
    
    # 4. Desired Attitude (ENU -> Body)
    # This aligns the rocket nose with the commanded acceleration vector
    q_ref = compute_desired_attitude(accel_cmd_enu, config)
    
    # 5. Attitude PID (Body Frame)
    # Error quaternion: represents the rotation from current orientation to desired orientation
    # q_error = q_ref * conjugate(q_real)
    q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))
    
    # Error mapping: e_roll=qx, e_pitch=qy, e_yaw=qz (Body Frame)
    error_vec = q_error[1:4]
    
    dt = config.control_dt_s
    controller["integral_error"] += error_vec * dt
    derivative = (error_vec - controller["previous_error"]) / dt
    controller["previous_error"] = error_vec
    
    # Roll damper: dampens p (roll rate) toward 0
    # Also tries to align roll to 0 if q_error[1] is significant
    u_roll = config.Kp_roll * error_vec[0] - config.Kd_attitude * w_body[0]
    
    # Pitch/Yaw control outputs (virtual control effort)
    u_pitch = (config.Kp_attitude * error_vec[1] + 
               config.Ki_attitude * controller["integral_error"][1] + 
               config.Kd_attitude * derivative[1])
               
    u_yaw = (config.Kp_attitude * error_vec[2] + 
             config.Ki_attitude * controller["integral_error"][2] + 
             config.Kd_attitude * derivative[2])
             
    # 6. Mixer to 4 rear fins (cross configuration)
    # Fin 1 (0 deg, right): u_yaw + u_roll
    # Fin 2 (90 deg, top): u_pitch + u_roll
    # Fin 3 (180 deg, left): -u_yaw + u_roll
    # Fin 4 (270 deg, bottom): -u_pitch + u_roll
    deltas = np.array([
        u_yaw + u_roll,
        u_pitch + u_roll,
       -u_yaw + u_roll,
       -u_pitch + u_roll
    ])
    
    # 7. Actuator limits (Delta max and Delta dot)
    # Use limits from controller state (passed from TOML via FinAdapter)
    try:
        delta_max = controller["delta_max_rad"]
        delta_dot_max = controller["delta_dot_max_rad_s"]
    except KeyError as e:
        raise RuntimeError(f"Controller state missing required limit: {e}. "
                           "Ensure rocket TOML defines control_actuation limits.") from e

    prev_deltas = controller.get("current_deltas", np.zeros(4))
    max_step = delta_dot_max * dt
    deltas = np.clip(deltas, prev_deltas - max_step, prev_deltas + max_step)
    
    # delta_max_rad
    deltas = np.clip(deltas, -delta_max, delta_max)
    
    # Update current state for GenericSurface adapter
    controller["current_deltas"] = deltas
    
    # Store history
    controller["deltas_history"][float(t)] = deltas
    
    return deltas

def build_controller(config):
    """
    Initializes controller state.
    """
    return {
        "integral_error": np.zeros(3),
        "previous_error": np.zeros(3),
        "current_deltas": np.zeros(4),
        "deltas_history": {}, # time -> [d1, d2, d3, d4] mapping
    }

def compute_desired_attitude(a_cmd_enu, config):
    """
    Returns ENU->Body q_ref aligning rocket nose (Body Z) with a_cmd_enu.
    """
    # Normalize desired direction
    norm = np.linalg.norm(a_cmd_enu)
    if norm < 1e-6:
        direction = np.array([0, 0, 1]) # Default to vertical
    else:
        direction = a_cmd_enu / norm
        
    # Rocket longitudinal axis in Body is [0, 0, 1]
    # We want to find rotation that takes ENU [0, 0, 1] to 'direction'?
    # No, q_ref is ENU -> Body. 
    # If rocket is aligned with 'direction', then Body Z in ENU is 'direction'.
    # That means the rotation ENU -> Body takes 'direction' to [0, 0, 1].
    
    q_ref = utils.quaternion_from_vectors(direction, np.array([0, 0, 1]))
    return q_ref

