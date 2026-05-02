import os

def load_config():
    """
    Returns execution parameters for the experiment.
    """
    config = type('Config', (), {})()
    
    # Timing
    config.control_dt_s = 0.02 # 50 Hz
    config.max_time_s = 300.0 # High enough to reach apogee
    
    # Guidance PD gains
    config.Kp_guidance = 1.0
    config.Kd_guidance = 0.5
    
    # Attitude PID gains (Pitch/Yaw)
    config.Kp_attitude = 5.0
    config.Ki_attitude = 0.1
    config.Kd_attitude = 1.0
    
    # Roll control gains
    config.Kp_attitude_roll = 2.0 # Proportional to roll error (qx)
    config.Kd_roll = 0.5          # Derivative / Damping gain (p)
    
    # Control activation
    config.control_start_delay_s = 3.0
    config.control_start_min_height_above_launch_m = 11.0 # rail_length (6) + 5
    
    # Control cutoff
    config.apogee_control_cutoff_delay_s = 0.5
    
    # Actuation limits (will be overriden by TOML if available in build_rocket)
    config.delta_max_rad = 0.349 # 20 degrees
    config.delta_dot_max_rad_s = 5.236 # 300 deg/s
    
    # Paths
    config.reference_path = "data/trajectory/vertical.csv"
    config.results_dir = "results"
    
    # Launch site / Rail 
    config.latitude = 42.3402247448
    config.longitude = -6.2713407985
    config.elevation_asl_m = 1000.0
    config.rail_length_m = 6.0
    config.heading_deg = 0.0
    config.inclination_deg = 90.0
    
    # Flags
    config.save_results = True
    config.show_plots = False
    config.interactive_rocket_plots = False # If True, shows rocket/motor/margin plots during creation
    
    return config
