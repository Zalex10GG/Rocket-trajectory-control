from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass
class Config:
    """
    Execution parameters and global configuration for the rocket simulation.
    This is the single source of truth for paths, environment, and controller gains.
    """
    # Timing
    control_dt_s: float = 0.02  # 50 Hz
    max_time_s: float = 600.0   # High enough to reach apogee

    # Guidance PD gains
    Kp_guidance: float = 0.5
    Kd_guidance: float = 1.0

    # Attitude PID gains (Pitch/Yaw)
    Kp_attitude: float = 0.00139
    Ki_attitude: float = 0.00234
    Kd_attitude: float = 0.000208

    # Roll damping gain
    Kp_roll: float = 0.3

    # Wind compensation gain
    K_wind_comp: float = 0.5

    # Dynamic pressure cutoff threshold (Pa)
    q_min_cutoff_pa: float = 100.0

    # Max guidance correction acceleration (m/s²)
    a_max_guidance_correction_m_s2: float = 8.0

    # Control activation
    control_start_delay_s: float = 1.0
    safety_margin_m: float = 1.0
    control_start_min_height_above_launch_m: float = 7.0

    # Control cutoff
    apogee_control_cutoff_delay_s: float = 0.5

    # Servo command smoothing
    actuator_command_filter_tau_s: float = 0.08

    # q-bar scheduled authority guardrail
    qbar_min_authority_pa: float = 500.0
    qbar_full_authority_pa: float = 5000.0
    delta_max_qbar_min_rad: float = 0.05
    qbar_high_authority_pa: float = 10000.0
    delta_max_qbar_high_rad: float = 0.104719755

    # Simulation termination
    terminate_on_apogee: bool = False

    # Paths (Centralized Truth)
    rocket_path: str = "data/rockets/leon_2.toml"
    motor_path: str = "data/motors/cesaroni_pro75_3g_3727l1050.csv"
    drag_path: str = "data/drag/leon_2_drag.csv"
    reference_path: str = "data/trajectory/vertical.csv"
    results_dir: str = "results"

    # Launch site / Rail
    latitude: float = 42.3402247448
    longitude: float = -6.2713407985
    elevation_asl_m: float = 1000.0
    rail_length_m: float = 6.0
    heading_deg: float = 0.0
    inclination_deg: float = 89.0

    # Environment
    launch_date: Tuple[int, int, int, int] = (2025, 5, 7, 12)
    atmosphere_type: str = "auto"  # "auto" | "Reanalysis" | "Forecast" | "standard_atmosphere"
    atmosphere_file: Optional[str] = None

    # Flags
    use_wind: bool = False
    save_results: bool = True
    show_plots: bool = False

def load_config() -> Config:
    """Returns a new instance of the Config dataclass."""
    return Config()
