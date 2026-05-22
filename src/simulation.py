"""
RocketPy-based simulation for controlled rocket flight.

This module handles the execution of the flight simulation and the exportation
of results. It uses RocketPy's internal controller infrastructure to integrate
closed-loop control.
"""

import numpy as np
import pandas as pd
import json
import os
from datetime import datetime
from rocketpy import Flight
from src.constants import CONTROL_SURFACE_NAME

# NOTE: Private API usage for controller integration
from rocketpy.control.controller import _Controller
import src.plots as plots
from src.controllers import fin_controller

def simulate_controlled_flight(rocket, environment, reference, controller, config):
    """
    Executes the closed-loop flight simulation using RocketPy's private
    controller infrastructure.

    Parameters
    ----------
    rocket : rocketpy.Rocket
        Built rocket with GenericSurface control fins.
    environment : rocketpy.Environment
        Atmospheric environment.
    reference : dict
        Loaded reference trajectory.
    controller : dict
        Mutable controller state (from ``build_controller``).
    config : Config
        Execution configuration.

    Returns
    -------
    list[dict]
        Flight history records with truthful delta reconstruction.
    """

    def controller_callback(t, sampling_rate, state, state_history,
                            observed_vars, interactive_objs, sensors, env):
        # RocketPy state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
        fin_controller(t, state, controller, config, reference, environment=env)
        return None

    # Clear existing controllers to avoid accumulation
    rocket._controllers = [] 
    
    # Locate the GenericSurface component for control interaction
    control_surf = None
    for item in rocket.aerodynamic_surfaces:
        if hasattr(item.component, 'name') and item.component.name == CONTROL_SURFACE_NAME:
            control_surf = item.component
            break
    
    ctrl_obj = _Controller(
        interactive_objects=[control_surf] if control_surf else [],
        controller_function=controller_callback,
        sampling_rate=1.0/config.control_dt_s,
        name="Fin Controller"
    )
    
    rocket._add_controllers(ctrl_obj)

    # Simulation Execution
    flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=config.rail_length_m,
        inclination=config.inclination_deg,
        heading=config.heading_deg,
        terminate_on_apogee=getattr(config, 'terminate_on_apogee', False),
        max_time=config.max_time_s,
        time_overshoot=False,
        verbose=False
    )
    
    _ = flight.apogee_time
    print(f"Simulation finished at t={flight.apogee_time:.2f}s. Extracting history...")
    
    # Extract history from solution
    sol = np.array(flight.solution)
    launch_pos_enu = sol[0, 1:4]
    
    # Precompute sorted controller times and deltas for lookup
    deltas_dict = controller["deltas_history"]
    if deltas_dict:
        ctrl_times_sorted = np.array(sorted(deltas_dict.keys()))
        ctrl_deltas_sorted = np.array([deltas_dict[k] for k in ctrl_times_sorted])
    else:
        ctrl_times_sorted = np.array([])
        ctrl_deltas_sorted = np.empty((0, 4))

    # Precompute q_ref lookup (controller-requested attitude quaternion)
    q_ref_dict = controller.get("q_ref_history", {})
    if q_ref_dict:
        qref_times_sorted = np.array(sorted(q_ref_dict.keys()))
        qref_values_sorted = np.array([q_ref_dict[k] for k in qref_times_sorted])
    else:
        qref_times_sorted = np.array([])
        qref_values_sorted = np.empty((0, 4))
    
    # Precompute q_dynamic lookup
    q_dynamic_dict = controller.get("q_dynamic_history", {})
    if q_dynamic_dict:
        qdyn_times_sorted = np.array(sorted(q_dynamic_dict.keys()))
        qdyn_values_sorted = np.array([q_dynamic_dict[k] for k in qdyn_times_sorted])
    else:
        qdyn_times_sorted = np.array([])
        qdyn_values_sorted = np.array([])
    
    history = []
    for i, t in enumerate(sol[:, 0]):
        state_vec = sol[i, 1:]
        
        # Truthful delta reconstruction: find the latest controller command with t_cmd <= t_sol
        if len(ctrl_times_sorted) > 0 and t <= ctrl_times_sorted[-1]:
            idx = np.searchsorted(ctrl_times_sorted, t, side='right')
            deltas = ctrl_deltas_sorted[idx - 1] if idx > 0 else np.zeros(4)
        else:
            deltas = np.zeros(4)
        
        # q_ref reconstruction: find the latest controller q_ref with t_cmd <= t_sol
        if len(qref_times_sorted) > 0 and t <= qref_times_sorted[-1]:
            idx_q = np.searchsorted(qref_times_sorted, t, side='right')
            q_ref = qref_values_sorted[idx_q - 1] if idx_q > 0 else np.full(4, np.nan)
        else:
            q_ref = np.full(4, np.nan)
            
        # q_dynamic reconstruction: find the latest q_dynamic with t_cmd <= t_sol
        if len(qdyn_times_sorted) > 0 and t <= qdyn_times_sorted[-1]:
            idx_qd = np.searchsorted(qdyn_times_sorted, t, side='right')
            q_dynamic = qdyn_values_sorted[idx_qd - 1] if idx_qd > 0 else 0.0
        else:
            q_dynamic = 0.0
            
        history.append({
            'time_s': float(t),
            'position_enu_m': state_vec[0:3] - launch_pos_enu, # Local ENU (launch=0,0,0)
            'position_asl_m': state_vec[0:3],                 # Absolute ASL
            'velocity_enu_m_s': state_vec[3:6],
            'attitude_quaternion': state_vec[6:10],
            'body_rates_rad_s': state_vec[10:13],
            'deltas': deltas,
            'q_ref': q_ref,  # Controller-requested attitude quaternion [w, x, y, z]
            'q_dynamic': q_dynamic,
            'mach': float(flight.mach_number(t)),
        })
    
    return history

def export_controller_diagnostics(controller, run_dir):
    """Persists controller diagnostics to a CSV artifact."""
    diag = controller.get("_diagnostics", [])
    if not diag:
        return None

    rows = []
    for d in diag:
        row = {
            "time_s": d["time_s"],
            "control_active": d["control_active"],
            "cutoff_reason": d["cutoff_reason"],
            "q_dynamic_pa": d["q_dynamic_pa"],
            "airspeed_m_s": d["airspeed_m_s"],
            "delta_limit_rad": d.get("delta_limit_rad", 0.0),
            "effective_cD": d.get("effective_cD", 0.0),
        }
        for key in ["raw_deltas_rad", "limited_deltas_rad",
                     "position_error_enu_m", "velocity_error_enu_m_s",
                     "attitude_error_quat", "commanded_accel_enu_m_s2"]:
            arr = d.get(key, [])
            for j, val in enumerate(arr):
                row[f"{key}_{j}"] = float(val)
        rows.append(row)

    df = pd.DataFrame(rows)
    path = os.path.join(run_dir, "controller_diagnostics.csv")
    df.to_csv(path, index=False)
    return path

def export_results(flight_history, reference, metrics, config, case_data, rocket=None, components=None, controller=None):
    """Saves metrics, history, and plots to results/<run_id>/"""
    if not config.save_results:
        print("Skipping results export (config.save_results is False)")
        return None

    # Run ID based on timestamp
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(config.results_dir, run_id)
    os.makedirs(run_dir, exist_ok=True)
    
    # Export rocket creation artifacts
    if rocket and components:
        from src.rocket_builder import export_rocket_creation_artifacts
        export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data)
    
    # Save metrics
    with open(os.path.join(run_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=4)
    
    # Save flight summary CSV
    if "summary" in metrics:
        pd.DataFrame([metrics["summary"]]).to_csv(os.path.join(run_dir, "flight_summary.csv"), index=False)
        
    # Save flight history
    if flight_history:
        flat_history = []
        for s in flight_history:
            flat_history.append({
                'time_s': s['time_s'],
                'x_local_m': s['position_enu_m'][0], 'y_local_m': s['position_enu_m'][1], 'z_local_m': s['position_enu_m'][2],
                'z_asl_m': s['position_asl_m'][2],
                'vx': s['velocity_enu_m_s'][0], 'vy': s['velocity_enu_m_s'][1], 'vz': s['velocity_enu_m_s'][2],
                'q0': s['attitude_quaternion'][0], 'q1': s['attitude_quaternion'][1], 'q2': s['attitude_quaternion'][2], 'q3': s['attitude_quaternion'][3],
                'p': s['body_rates_rad_s'][0], 'q': s['body_rates_rad_s'][1], 'r': s['body_rates_rad_s'][2],
                'delta1': s['deltas'][0], 'delta2': s['deltas'][1], 'delta3': s['deltas'][2], 'delta4': s['deltas'][3],
                'qref0': s['q_ref'][0], 'qref1': s['q_ref'][1], 'qref2': s['q_ref'][2], 'qref3': s['q_ref'][3],
                'q_dynamic_pa': s.get('q_dynamic', 0.0),
                'mach': s.get('mach', 0.0),
            })
        pd.DataFrame(flat_history).to_csv(os.path.join(run_dir, "flight_history.csv"), index=False)
    
    # Export controller diagnostics
    if controller is not None:
        export_controller_diagnostics(controller, run_dir)

    # Generate plots
    plots.generate_all_plots(flight_history, reference, metrics, config, run_dir, controller_state=controller)
    if rocket and components:
        plots.generate_rocket_creation_plots(rocket, components, run_dir)
    
    print(f"Results exported to {run_dir}")
    return run_dir
