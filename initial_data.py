import os
import toml
import pandas as pd

def load_initial_case_data():
    """
    Loads the physical definition of the nominal case.
    """
    data = {
        # Launch site and environment (English names)
        "latitude": 42.3402247448,
        "longitude": -6.2713407985,
        "elevation_asl_m": 1000.0,
        "rail_length_m": 6.0,
        "heading_deg": 0.0,
        "inclination_deg": 90.0,
        
        # Paths to external assets
        "rocket_path": "data/rockets/leon_2.toml",
        "motor_path": "data/motors/cesaroni_pro75_3g_3727l1050.csv",
        "drag_path": "data/drag/leon_2_drag.csv",
        "trajectory_path": "data/trajectory/vertical.csv",
    }
    
    # Load rocket TOML to get aerodynamic and geometric params
    if os.path.exists(data["rocket_path"]):
        with open(data["rocket_path"], "r") as f:
            data["rocket_params"] = toml.load(f)
    else:
        # Fallback/Default values for Leon 2 if file doesn't exist yet
        data["rocket_params"] = {
            "name": "Leon 2",
            "diameter_m": 0.08,
            "reference_area_m2": 0.005026,
            "reference_length_m": 0.08,
            "fin_aerodynamic_center_x_m": -1.2, # Relative to nose
            "fin_aerodynamic_center_y_m": 0.06, # Radial arm
            "cN_delta_per_rad": 2.0,
            "cl_delta_per_rad": 0.1,
            "cd_delta_per_rad": 0.0,
            "k_drag_induced": 0.5,
            "delta_max_rad": 0.26, # ~15 deg
            "delta_dot_max_rad_s": 5.0,
        }
    
    return data
