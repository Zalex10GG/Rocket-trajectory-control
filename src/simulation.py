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

# NOTE: Private API usage - see module docstring
from rocketpy.control.controller import _Controller  # noqa: API private
import src.plots as plots
from src.controllers import fin_controller

def simulate_controlled_flight(rocket, environment, reference, controller, config):
    """
    Executes the truthful closed-loop flight simulation using RocketPy's private controller infrastructure.
    """
    # 1. Define controller callback
    def controller_callback(t, sampling_rate, state, state_history, observed_vars, interactive_objs, sensors, env):
        # RocketPy state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
        # Same as our fin_controller expectation.
        fin_controller(t, state, controller, config, reference)
        return None # Controller function should return None

    # 2. Add controller to rocket via private infrastructure
    # Clear existing controllers to avoid accumulation
    rocket._controllers = [] 
    
    # No AirBrakes involved. We pass the GenericSurface as an interactive object if needed.
    # We iterate over aerodynamic_surfaces (which are component_tuples) to find our surface.
    control_surf = None
    for item in rocket.aerodynamic_surfaces:
        if hasattr(item.component, 'name') and item.component.name == "Control Fin Deflection Increment":
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
        terminate_on_apogee=True, 
        max_time=config.max_time_s,
        time_overshoot=False, # High precision for control
        verbose=False         # Reduce console noise
    )
    
    _ = flight.apogee_time
    
    print(f"Simulation finished at t={flight.apogee_time:.2f}s. Extracting history...")
    
    # 4. Extract history from solution
    sol = np.array(flight.solution)
    history = []
    
    # Normalization factor: Launch position in ENU
    # In RocketPy, the initial position is [0, 0, elevation_asl]
    # We want position_enu_m to be [0, 0, 0] at launch.
    launch_pos_enu = sol[0, 1:4]
    
    # Get deltas from controller history
    # RocketPy might have evaluated controller at more/less points than solution nodes
    # depending on the ODE solver.
    
    for i, t in enumerate(sol[:, 0]):
        state_vec = sol[i, 1:]
        
        # Recover deltas for this timestamp from the controller history
        # (which was populated DURING simulation)
        # We find the closest timestamp in controller["deltas_history"]
        ctrl_times = np.array(list(controller["deltas_history"].keys()))
        if len(ctrl_times) > 0:
            idx = (np.abs(ctrl_times - t)).argmin()
            deltas = controller["deltas_history"][ctrl_times[idx]]
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

def export_results(flight_history, reference, metrics, config, rocket=None, components=None):
    """
    Saves results to results/<run_id>/
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
        export_rocket_creation_artifacts(rocket, components, run_dir, config)
    
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
        
    # 5. Export plots to run directory
    plots.generate_all_plots(flight_history, reference, metrics, config, run_dir)
    
    print(f"Results exported to {run_dir}")
    return run_dir
