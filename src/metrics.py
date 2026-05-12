"""
Tracking performance metrics for controlled rocket flight.

Computes position tracking errors, actuator saturation statistics, and
dynamic-pressure / wind diagnostics for the control phase window.

Distinguishes between:
- **active-control window**: samples where the controller was actively
  commanding (based on diagnostics, not just nonzero deltas).
- **ascent window**: from control activation to apogee (may include
  post-cutoff coasting).
"""

import numpy as np


def compute_tracking_metrics(flight_history, reference, config, controller_state=None):
    """
    Computes quantitative tracking performance metrics for trajectory tracking.

    Focuses on the active-control phase window for performance metrics.  Includes
    diagnostic data on dynamic pressure, wind speed, airspeed, actuator
    saturation, and control-induced drag to support tuning and acceptance criteria.

    Parameters
    ----------
    flight_history : list[dict]
        Flight state records from ``simulate_controlled_flight``.
    reference : dict
        Loaded reference trajectory.
    config : Config
        Execution configuration.
    controller_state : dict, optional
        Mutable controller state dictionary with diagnostic fields.

    Returns
    -------
    dict
        Metrics dictionary with keys for errors, saturation, windows, and summary.
    """
    if not flight_history:
        return {"error": "No flight history data"}

    import src.utils as utils
    import src.reference as ref_mod

    # Identify control windows (Task 5: split active vs ascent)
    ctrl_start_idx, apogee_idx = utils.get_control_window_indices(
        flight_history, controller_state
    )
    active_start_idx, active_end_idx = utils.get_active_control_window_indices(
        flight_history, controller_state
    )

    # Use active-control window for control metrics
    ctrl_history = flight_history[active_start_idx:active_end_idx+1]

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

    # --- Lateral error (XY plane) ---
    lateral_errors = np.sqrt(errors_x**2 + errors_y**2)

    metrics = {
        "ctrl_mae_3d_m": float(np.mean(errors_3d)),
        "ctrl_rmse_3d_m": float(np.sqrt(np.mean(errors_3d**2))),
        "ctrl_max_error_3d_m": float(np.max(errors_3d)),
        "ctrl_mae_x_m": float(np.mean(np.abs(errors_x))),
        "ctrl_mae_y_m": float(np.mean(np.abs(errors_y))),
        "ctrl_mae_z_m": float(np.mean(np.abs(errors_z))),
        "ctrl_rmse_lateral_m": float(np.sqrt(np.mean(lateral_errors**2))),
        "ctrl_max_lateral_m": float(np.max(lateral_errors)),
        "max_fin_deflection_deg": float(np.max(abs_deltas_deg)),
    }

    # --- Active-control window timing (Task 5) ---
    times = np.array([s['time_s'] for s in flight_history])
    if active_start_idx < len(times) and active_end_idx < len(times):
        metrics["control_active_start_s"] = float(times[active_start_idx])
        metrics["control_active_end_s"] = float(times[active_end_idx])
        metrics["control_active_duration_s"] = float(
            times[active_end_idx] - times[active_start_idx]
        )
    else:
        metrics["control_active_start_s"] = 0.0
        metrics["control_active_end_s"] = 0.0
        metrics["control_active_duration_s"] = 0.0

    # --- Saturation ratio (active-control diagnostics, Task 5) ---
    sat_count = 0
    total_ctrl_samples = len(deltas)
    diag = controller_state.get("_diagnostics", []) if controller_state else []
    active_diag = [d for d in diag if d.get("control_active", False)]
    if active_diag:
        diag_deltas = np.array([d["limited_deltas_rad"] for d in active_diag])
        diag_limits = np.array([max(d.get("delta_limit_rad", 0.0), 1e-9) for d in active_diag])
        sat_mask_per_sample = np.array([
            np.any(np.abs(diag_deltas[i]) >= 0.95 * diag_limits[i])
            for i in range(len(active_diag))
        ])
        sat_count = int(np.sum(sat_mask_per_sample))
        metrics["fin_saturation_ratio"] = float(sat_count / len(active_diag))
    elif total_ctrl_samples > 0 and controller_state and "delta_max_rad" in controller_state:
        dmax = controller_state["delta_max_rad"]
        sat_mask_per_sample = np.any(np.abs(deltas) >= 0.95 * dmax, axis=1)
        sat_count = int(np.sum(sat_mask_per_sample))
        metrics["fin_saturation_ratio"] = float(sat_count / total_ctrl_samples)
    else:
        metrics["fin_saturation_ratio"] = 0.0

    # --- Cumulative saturation time from active-control diagnostics ---
    if active_diag:
        diag_times = np.array([d["time_s"] for d in active_diag])
        if len(diag_times) > 1:
            dt_arr = np.diff(diag_times)
            dt_arr = np.append(dt_arr, dt_arr[-1])
            metrics["saturation_time_s"] = float(np.sum(dt_arr[sat_mask_per_sample]))
            ctrl_duration = float(diag_times[-1] - diag_times[0])
            metrics["active_control_duration_s"] = ctrl_duration
            metrics["saturation_time_ratio"] = (
                metrics["saturation_time_s"] / ctrl_duration if ctrl_duration > 0 else 0.0
            )
        else:
            metrics["saturation_time_s"] = 0.0
            metrics["active_control_duration_s"] = 0.0
            metrics["saturation_time_ratio"] = 0.0
    elif total_ctrl_samples > 0:
        ctrl_times = np.array([s['time_s'] for s in ctrl_history])
        if len(ctrl_times) > 1:
            dt_arr = np.diff(ctrl_times)
            dt_arr = np.append(dt_arr, dt_arr[-1] if len(dt_arr) > 0 else 0.02)
            if controller_state and "delta_max_rad" in controller_state:
                dmax = controller_state["delta_max_rad"]
                sat_time_mask = np.any(np.abs(deltas) >= 0.95 * dmax, axis=1)
            else:
                sat_time_mask = np.zeros(len(deltas), dtype=bool)

            metrics["saturation_time_s"] = float(np.sum(dt_arr[sat_time_mask]))
            ctrl_duration = float(ctrl_times[-1] - ctrl_times[0])
            metrics["active_control_duration_s"] = ctrl_duration
            metrics["saturation_time_ratio"] = (
                metrics["saturation_time_s"] / ctrl_duration if ctrl_duration > 0 else 0.0
            )
        else:
            metrics["saturation_time_s"] = 0.0
            metrics["active_control_duration_s"] = 0.0
            metrics["saturation_time_ratio"] = 0.0
    else:
        metrics["saturation_time_s"] = 0.0
        metrics["active_control_duration_s"] = 0.0
        metrics["saturation_time_ratio"] = 0.0

    # --- Control coefficient diagnostics (Task 10) ---
    if controller_state:
        if active_diag:
            cD_values = [d.get("effective_cD", 0.0) for d in active_diag]
            metrics["max_control_cD"] = float(max(cD_values))
            metrics["mean_control_cD"] = float(np.mean(cD_values))
        else:
            metrics["max_control_cD"] = 0.0
            metrics["mean_control_cD"] = 0.0

    # --- Duplicate callback counter ---
    if controller_state:
        metrics["duplicate_callback_count"] = int(
            controller_state.get("_duplicate_callback_count", 0)
        )

    # --- Wind / dynamic pressure diagnostics (from controller state) ---
    if controller_state:
        metrics["last_dynamic_pressure_pa"] = float(controller_state.get("last_q_dynamic", 0.0))
        metrics["last_airspeed_m_s"] = float(controller_state.get("last_airspeed", 0.0))
        wind = controller_state.get("last_wind_enu", np.zeros(3))
        metrics["last_wind_speed_m_s"] = float(np.linalg.norm(wind))

    # --- Flight Summary Events (Full Flight) ---
    pos_local = np.array([s['position_enu_m'] for s in flight_history])
    pos_asl = np.array([s['position_asl_m'] for s in flight_history])
    vel_real = np.array([s['velocity_enu_m_s'] for s in flight_history])
    speeds = np.linalg.norm(vel_real, axis=1)

    apogee_idx_full = int(np.argmax(pos_local[:, 2]))

    metrics["summary"] = {
        "launch_altitude_asl_m": float(pos_asl[0, 2]),
        "max_altitude_asl_m": float(pos_asl[apogee_idx_full, 2]),
        "max_altitude_local_m": float(pos_local[apogee_idx_full, 2]),
        "time_of_apogee_s": float(times[apogee_idx_full]),
        "final_time_s": float(times[-1]),
        "max_speed_m_s": float(np.max(speeds)),
        "control_active_start_s": metrics["control_active_start_s"],
        "control_active_end_s": metrics["control_active_end_s"],
        "control_active_duration_s": metrics["control_active_duration_s"],
        "ascent_window_start_s": float(times[ctrl_start_idx]) if ctrl_start_idx < len(times) else 0.0,
        "ascent_window_end_s": float(times[apogee_idx]) if apogee_idx < len(times) else 0.0,
        "ascent_window_duration_s": float(times[apogee_idx] - times[ctrl_start_idx]) if apogee_idx < len(times) and ctrl_start_idx < len(times) else 0.0,
        "max_fin_deflection_deg": metrics["max_fin_deflection_deg"],
        "fin_saturation_ratio": metrics["fin_saturation_ratio"],
        "ctrl_rmse_lateral_m": metrics["ctrl_rmse_lateral_m"],
        "max_control_cD": metrics.get("max_control_cD", 0.0),
        "mean_control_cD": metrics.get("mean_control_cD", 0.0),
        "duplicate_callback_count": metrics.get("duplicate_callback_count", 0),
    }

    return metrics
