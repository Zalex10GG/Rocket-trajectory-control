def load_config():
    """
    Returns execution parameters for the experiment.
    """
    config = type('Config', (), {})()
    
    # Timing
    config.control_dt_s = 0.02 # 50 Hz
    config.max_time_s = 600.0 # High enough to reach apogee
    
    # Guidance PD gains
    config.Kp_guidance = 1.0
    config.Kd_guidance = 0.5
    
    # Attitude PID gains (Pitch/Yaw)
    config.Kp_attitude = 5.0
    config.Ki_attitude = 0.1
    config.Kd_attitude = 1.0
    
    # Roll damping gain
    config.Kp_roll = 0.5
    
    # Control activation
    config.control_start_delay_s = 3.0
    config.safety_margin_m = 5.0
    # config.control_start_min_height_above_launch_m is derived in build_rocket

    # Control cutoff
    config.apogee_control_cutoff_delay_s = 0.5
    
    # Paths
    config.reference_path = "data/trajectory/vertical.csv"
    config.results_dir = "results"
    
    # Launch site / Rail (English names)
    config.latitude = 42.3402247448
    config.longitude = -6.2713407985
    config.elevation_asl_m = 1000.0
    config.rail_length_m = 6.0
    config.heading_deg = 0.0
    config.inclination_deg = 90.0
    
    # Flags
    config.save_results = True
    config.show_plots = False
    
    return config
