import os
import toml

def load_initial_case_data(config):
    """
    Loads the physical definition of the nominal case.
    """
    data = {
        # Launch site and environment (English names)
        # Source of truth: config.py
        "latitude": config.latitude,
        "longitude": config.longitude,
        "elevation_asl_m": config.elevation_asl_m,
        "rail_length_m": config.rail_length_m,
        "heading_deg": config.heading_deg,
        "inclination_deg": config.inclination_deg,
        
        # Paths to external assets
        "rocket_path": "data/rockets/leon_2.toml",
        "motor_path": "data/motors/cesaroni_pro75_3g_3727l1050.csv",
        "drag_path": "data/drag/leon_2_drag.csv",
        "trajectory_path": "data/trajectory/vertical.csv",
    }
    
    # Load rocket TOML to get aerodynamic and geometric params
    if not os.path.exists(data["rocket_path"]):
        raise FileNotFoundError(f"Critical error: Rocket definition file not found at {data['rocket_path']}. "
                                "Leon 2 physical data is required to run the simulation.")

    with open(data["rocket_path"], "r") as f:
        data["rocket_params"] = toml.load(f)
    
    return data
