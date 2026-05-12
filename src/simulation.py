"""
RocketPy-based simulation for controlled rocket flight.

LIMITATION KNOWN: RocketPy v1.12.1 does NOT expose a public API for closed-loop
fin control. The library only supports:
- Passive aerodynamic surfaces (fixed geometry)
- AirBrakes (simple deployment profile, not arbitrary closed-loop)

The internal infrastructure used below (_Controller, rocket._add_controllers) is
private and may break between versions. This is a known limitation tracked in:
- RocketPy Issue #915: "Active fins"
- RocketPy Issue #917: "Per-Step Data & Individual Fin Control"

A public API for active fin control has been requested but is not yet available
as of v1.12.1. This solution uses private infrastructure and may require
updates when RocketPy releases a public control API.

Reference: https://github.com/RocketPy-Team/RocketPy/issues/917
"""

import numpy as np
import pandas as pd
import json
import os
from datetime import datetime
from rocketpy import Flight
from src.constants import CONTROL_SURFACE_NAME

# NOTE: Private API usage - see module docstring
from rocketpy.control.controller import _Controller
import src.plots as plots
from src.controllers import fin_controller

def simulate_controlled_flight(rocket, environment, reference, controller, config):
    """
    Executes the closed-loop flight simulation using RocketPy's private
    controller infrastructure.

    The RocketPy ``_Controller`` callback receives the ``Environment`` object
    as its last argument (8-parameter signature).  This is forwarded to
    ``fin_controller`` so it can query local air density, wind velocity, and
    gravity for dynamic-pressure cutoff, wind compensation, and gravity
    feedforward.

    The controller is idempotent per effective timestamp: duplicate callbacks
    at the same simulation time return the previously computed command without
    advancing controller state.

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

    # 2. Add controller to rocket via private infrastructure
    # Clear existing controllers to avoid accumulation
    rocket._controllers = [] 
    
    # No AirBrakes involved. We pass the GenericSurface as an interactive object if needed.
    # We iterate over aerodynamic_surfaces (which are component_tuples) to find our surface.
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

    # 3. Simulation Execution
    flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=config.rail_length_m,
        inclination=config.inclination_deg,
        heading=config.heading_deg,
        terminate_on_apogee=getattr(config, 'terminate_on_apogee', False),
        max_time=config.max_time_s,
        time_overshoot=False, # High precision for control
        verbose=False         # Reduce console noise
    )
    
    _ = flight.apogee_time
    
    print(f"Simulation finished at t={flight.apogee_time:.2f}s. Extracting history...")
    
    # 4. Extract history from solution with truthful delta reconstruction
    # Use "latest command at or before solution time" — never nearest future command.
    sol = np.array(flight.solution)
    history = []
    
    # Normalization factor: Launch position in ENU
    # In RocketPy, the initial position is [0, 0, elevation_asl]
    # We want position_enu_m to be [0, 0, 0] at launch.
    launch_pos_enu = sol[0, 1:4]
    
    # Precompute sorted controller times and deltas for O(log n) lookup per timestep.
    deltas_dict = controller["deltas_history"]
    if deltas_dict:
        ctrl_times_sorted = np.array(sorted(deltas_dict.keys()))
        ctrl_deltas_sorted = np.array([deltas_dict[k] for k in ctrl_times_sorted])
    else:
        ctrl_times_sorted = np.array([])
        ctrl_deltas_sorted = np.empty((0, 4))
    
    for i, t in enumerate(sol[:, 0]):
        state_vec = sol[i, 1:]
        
        # Truthful delta reconstruction: find the latest controller command
        # with command_time <= solution_time.  If no such command exists,
        # export zeros.  This prevents assigning future commands to past states.
        if len(ctrl_times_sorted) > 0:
            # searchsorted with side='right' gives index of first element > t
            idx = np.searchsorted(ctrl_times_sorted, t, side='right')
            if idx > 0:
                deltas = ctrl_deltas_sorted[idx - 1]
            else:
                # No command at or before this solution time
                deltas = np.zeros(4)
        else:
            deltas = np.zeros(4)
            
        history.append({
            'time_s': float(t),
            'position_enu_m': state_vec[0:3] - launch_pos_enu, # Local ENU (launch=0,0,0)
            'position_asl_m': state_vec[0:3],                 # Absolute ASL for reference
            'velocity_enu_m_s': state_vec[3:6],
            'attitude_quaternion': state_vec[6:10],
            'body_rates_rad_s': state_vec[10:13],
            'deltas': deltas
        })
    
    return history


def export_controller_diagnostics(controller, run_dir):
    """
    Persists controller diagnostics to a CSV artifact in the result directory.

    Each row corresponds to one effective controller sample (or a duplicate
    callback detection).  This allows post-hoc auditing of the control path
    without re-running the simulation.

    Parameters
    ----------
    controller : dict
        Mutable controller state with ``_diagnostics`` list.
    run_dir : str
        Path to the result directory for this run.

    Returns
    -------
    str or None
        Path to the diagnostics file, or None if no diagnostics available.
    """
    diag = controller.get("_diagnostics", [])
    if not diag:
        print("WARNING: No controller diagnostics available to export.")
        return None

    # Flatten nested lists for CSV columns
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
        # Flatten array fields
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
    print(f"Controller diagnostics exported: {path}")
    return path


def export_results(flight_history, reference, metrics, config, case_data, rocket=None, components=None, controller=None):
    """
    Saves results to results/<run_id>/

    Parameters
    ----------
    flight_history : list[dict]
        Flight state records.
    reference : dict
        Loaded reference trajectory.
    metrics : dict
        Computed metrics dictionary.
    config : Config
        Execution configuration.
    case_data : dict
        Physical case data.
    rocket : rocketpy.Rocket, optional
        Built rocket object.
    components : dict, optional
        Component dictionary from rocket_builder.
    controller : dict, optional
        Controller state dict with diagnostics.
    """
    if not config.save_results:
        print("Skipping results export (config.save_results is False)")
        return None

    # Resolve run_id with microsecond precision to avoid collisions
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = os.path.join(config.results_dir, run_id)
    os.makedirs(run_dir, exist_ok=True)
    
    # Export rocket creation artifacts if provided
    if rocket and components:
        from src.rocket_builder import export_rocket_creation_artifacts
        export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data)
    
    # Save metrics
    with open(os.path.join(run_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=4)
    
    # Save flight summary CSV
    if "summary" in metrics:
        summary_df = pd.DataFrame([metrics["summary"]])
        summary_df.to_csv(os.path.join(run_dir, "flight_summary.csv"), index=False)
        
    # Save flight history
    if flight_history:
        flat_history = []
        for s in flight_history:
            item = {
                'time_s': s['time_s'],
                'x_local_m': s['position_enu_m'][0],
                'y_local_m': s['position_enu_m'][1],
                'z_local_m': s['position_enu_m'][2],
                'z_asl_m': s['position_asl_m'][2],
                'vx': s['velocity_enu_m_s'][0],
                'vy': s['velocity_enu_m_s'][1],
                'vz': s['velocity_enu_m_s'][2],
                'q0': s['attitude_quaternion'][0],
                'q1': s['attitude_quaternion'][1],
                'q2': s['attitude_quaternion'][2],
                'q3': s['attitude_quaternion'][3],
                'p': s['body_rates_rad_s'][0],
                'q': s['body_rates_rad_s'][1],
                'r': s['body_rates_rad_s'][2],
                'delta1': s['deltas'][0],
                'delta2': s['deltas'][1],
                'delta3': s['deltas'][2],
                'delta4': s['deltas'][3]
            }
            flat_history.append(item)
        df = pd.DataFrame(flat_history)
        df.to_csv(os.path.join(run_dir, "flight_history.csv"), index=False)
    
    # Export controller diagnostics (Task 3)
    if controller is not None:
        export_controller_diagnostics(controller, run_dir)

    # 5. Export plots to run directory
    plots.generate_all_plots(
        flight_history, reference, metrics, config, run_dir, controller_state=controller
    )

    # 6. RocketPy static plots (rocket diagram, static margin, motor thrust)
    if rocket and components:
        plots.generate_rocket_creation_plots(rocket, components, run_dir)
    
    print(f"Results exported to {run_dir}")
    return run_dir
