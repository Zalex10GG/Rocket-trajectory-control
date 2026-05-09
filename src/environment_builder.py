import numpy as np
from rocketpy import Environment

def build_environment(case_data, config):
    """
    Constructs the RocketPy Environment.

    Note: The 'config' argument is currently reserved for future enhancements (e.g., custom
    weather settings or complex atmosphere models) and is not used in the current implementation.
    """
    env = Environment(
        latitude=case_data["latitude"],
        longitude=case_data["longitude"],
        elevation=case_data["elevation_asl_m"]
    )
    
    # Set fixed date for reproducibility
    env.set_date((2026, 4, 28, 12))
    
    # ISA Atmosphere (default in RocketPy if no ensemble/forecast is set)
    env.set_atmospheric_model(type='standard_atmosphere')
    
    return env
