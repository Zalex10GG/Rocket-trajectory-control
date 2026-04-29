import numpy as np

def compute_tracking_metrics(flight_history, reference, config):
    """
    Computes quantitative tracking performance metrics for trajectory tracking.
    Focuses on the control phase window for performance metrics.
    """
    if not flight_history:
        return {"error": "No flight history data"}
        
    import src.utils as utils
    import src.reference as ref_mod
    
    # Identify control window
    start_idx, end_idx = utils.get_control_window_indices(flight_history)
    ctrl_history = flight_history[start_idx:end_idx+1]
    
    errors_3d = []
    errors_x = []
    errors_y = []
    errors_z = []
    deltas = []
    
    for sample in ctrl_history:
        t = sample['time_s']
        ref = ref_mod.sample_reference(reference, t)
        
        # Position error in ENU
        e_vec = ref['position_enu_m'] - sample['position_enu_m']
        dist_error = np.linalg.norm(e_vec)
        
        errors_3d.append(dist_error)
        errors_x.append(e_vec[0])
        errors_y.append(e_vec[1])
        errors_z.append(e_vec[2])
        deltas.append(sample['deltas'])
        
    errors_3d = np.array(errors_3d)
    errors_x = np.array(errors_x)
    errors_y = np.array(errors_y)
    errors_z = np.array(errors_z)
    deltas = np.array(deltas)
    
    abs_deltas_deg = np.abs(deltas) * 180.0 / np.pi
    
    metrics = {
        "ctrl_mae_3d_m": float(np.mean(errors_3d)),
        "ctrl_rmse_3d_m": float(np.sqrt(np.mean(errors_3d**2))),
        "ctrl_max_error_3d_m": float(np.max(errors_3d)),
        "ctrl_mae_x_m": float(np.mean(np.abs(errors_x))),
        "ctrl_mae_y_m": float(np.mean(np.abs(errors_y))),
        "ctrl_mae_z_m": float(np.mean(np.abs(errors_z))),
        "max_fin_deflection_deg": float(np.max(abs_deltas_deg)),
    }
    
    # Calculate saturation ratio (in control phase)
    sat_count = 0
    total_ctrl_samples = len(deltas)
    if total_ctrl_samples > 0:
        # Consider saturated if ANY fin is at > 95% of max
        sat_mask = np.any(np.abs(deltas) >= 0.95 * config.delta_max_rad, axis=1)
        sat_count = np.sum(sat_mask)
        metrics["fin_saturation_ratio"] = float(sat_count / total_ctrl_samples)
    else:
        metrics["fin_saturation_ratio"] = 0.0
    
    # --- Flight Summary Events (Full Flight) ---
    times = np.array([s['time_s'] for s in flight_history])
    pos_local = np.array([s['position_enu_m'] for s in flight_history])
    pos_asl = np.array([s['position_asl_m'] for s in flight_history])
    vel_real = np.array([s['velocity_enu_m_s'] for s in flight_history])
    speeds = np.linalg.norm(vel_real, axis=1)
    
    apogee_idx_full = np.argmax(pos_local[:, 2])
    
    metrics["summary"] = {
        "launch_altitude_asl_m": float(pos_asl[0, 2]),
        "max_altitude_asl_m": float(pos_asl[apogee_idx_full, 2]),
        "max_altitude_local_m": float(pos_local[apogee_idx_full, 2]),
        "time_of_apogee_s": float(times[apogee_idx_full]),
        "final_time_s": float(times[-1]),
        "max_speed_m_s": float(np.max(speeds)),
        "control_phase_start_s": float(times[start_idx]),
        "control_phase_end_s": float(times[end_idx]),
        "control_phase_duration_s": float(times[end_idx] - times[start_idx]),
        "max_fin_deflection_deg": metrics["max_fin_deflection_deg"],
        "fin_saturation_ratio": metrics["fin_saturation_ratio"]
    }
    
    return metrics
