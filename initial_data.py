import toml

def load_initial_case_data(config):
    """
    Loads the physical rocket definition from the TOML file specified in config.
    Returns a dictionary with the rocket parameters and asset paths.
    """
    # Load rocket TOML to get aerodynamic and geometric params
    with open(config.rocket_path, "r") as f:
        rocket_params = toml.load(f)
    
    return {
        "rocket_params": rocket_params,
        "motor_path": config.motor_path,
        "drag_path": config.drag_path,
        "reference_path": config.reference_path,
        "rail_length_m": config.rail_length_m,
    }
