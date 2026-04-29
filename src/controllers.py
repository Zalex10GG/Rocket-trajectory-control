import numpy as np
import src.utils as utils

def fin_controller(t, state, controller, config, reference):
    """
    Callback for RocketPy GenericSurface integration.
    t: time (s)
    state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz] (ENU)
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
    z_local = z_asl - config.elevation_asl_m
    vz = vel[2]
    
    if t < config.control_start_delay_s or z_local < config.control_start_min_height_above_launch_m or vz <= 0:
        deltas = np.zeros(4)
        controller["deltas_history"][float(t)] = deltas
        return deltas

    # 3. Guidance: Trajectory tracking (Local ENU)
    from src.reference import sample_reference
    ref_sample = sample_reference(reference, t)
    pos_ref = ref_sample['position_enu_m'] # Reference is already local ENU (0,0,0)
    vel_ref = ref_sample['velocity_enu_m_s']
    
    # Normalize current position to local ENU for tracking
    pos_local = pos - np.array([0, 0, config.elevation_asl_m])
    
    # Simple PD guidance to get desired acceleration/direction in ENU
    # We want to align the velocity vector with the reference path
    # Desired acceleration is used to define the target pointing vector
    accel_cmd_enu = config.Kp_guidance * (pos_ref - pos_local) + config.Kd_guidance * (vel_ref - vel)
    # Add vertical component to bias the pointing vector upwards
    # This ensures the rocket maintains an upward orientation during tracking
    accel_cmd_enu += np.array([0, 0, 9.81]) 
    
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
    u_roll = -config.Kp_roll * w_body[0]
    
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
    
    # 7. Saturate
    deltas = np.clip(deltas, -config.delta_max_rad, config.delta_max_rad)
    
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

# Unused in main loop, preserved for reference
# def update_attitude_pid(state, q_ref, controller_state, config):
#     """
#     Attitude PID in Body frame.
#     q_error = q_ref * conjugate(q_real)
#     H4: mapping e_roll=x, e_pitch=y, e_yaw=z
#     """
#     q_real = state['attitude_quaternion'] # [w, x, y, z] ENU -> Body
#     
#     # H4: q_error = q_ref * conjugate(q_real)
#     q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))
#     
#     # H4: PID vector-part mapping: e_roll=x, e_pitch=y, e_yaw=z
#     # q_error = [w, x, y, z]
#     e_roll = q_error[1]
#     e_pitch = q_error[2]
#     e_yaw = q_error[3]
#     
#     error_vec = np.array([e_roll, e_pitch, e_yaw])
#     
#     dt = config.control_dt_s
#     
#     # Update integral
#     controller_state["integral_error"] += error_vec * dt
#     
#     # Derivative
#     derivative = (error_vec - controller_state["previous_error"]) / dt
#     
#     # PID output (virtual commands)
#     # Using separate gains if needed, but for V1 we use same for pitch/yaw
#     u_roll = -config.Kp_roll * state['body_rates_rad_s'][0] # Simple damper
#     
#     u_pitch = (config.Kp_attitude * e_pitch + 
#                config.Ki_attitude * controller_state["integral_error"][1] + 
#                config.Kd_attitude * derivative[1])
#                
#     u_yaw = (config.Kp_attitude * e_yaw + 
#              config.Ki_attitude * controller_state["integral_error"][2] + 
#              config.Kd_attitude * derivative[2])
#     
#     controller_state["previous_error"] = error_vec
#     
#     return {
#         "u_pitch": u_pitch,
#         "u_yaw": u_yaw,
#         "u_roll": u_roll
#     }
