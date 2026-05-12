from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class Config:
    # Timing
    control_dt_s: float = 0.02  # 50 Hz
    max_time_s: float = 600.0  # High enough to reach apogee

    # Guidance PD gains (conservative / over-damped for wind rejection)
    Kp_guidance: float = 0.5
    Kd_guidance: float = 1.0

    # Attitude PID gains (Pitch/Yaw) — conservative, prioritise stability
    Kp_attitude: float = 1.0
    Ki_attitude: float = 0.0  # No integral action; anti-windup + feedforward replace it
    Kd_attitude: float = 0.2

    # Roll damping gain
    Kp_roll: float = 0.3

    # Wind compensation gain (feedforward on local wind velocity)
    K_wind_comp: float = 0.5

    # Anti-windup tracking time constant (seconds). Back-calculation gain = 1/T_t.
    anti_windup_T_t: float = 0.5

    # Dynamic pressure cutoff threshold (Pa).
    # Below this value fins have insufficient aerodynamic authority.
    q_min_cutoff_pa: float = 100.0

    # Max guidance correction acceleration (m/s²).  Clips the PD + wind
    # correction component (excluding gravity) to prevent commanding
    # physically unachievable attitude changes.
    a_max_guidance_correction_m_s2: float = 8.0

    # Control activation
    control_start_delay_s: float = 3.0
    safety_margin_m: float = 5.0
    control_start_min_height_above_launch_m: float = 10.0

    # Control cutoff
    apogee_control_cutoff_delay_s: float = 0.5

    # Servo command smoothing.  This first-order filter is applied before the
    # existing rate and position limits so high-frequency attitude jitter does
    # not become instantaneous fin chatter.
    actuator_command_filter_tau_s: float = 0.08

    # q-bar scheduled authority guardrail (Task 9)
    # Linear ramp: at qbar_min_authority_pa the max deflection is
    # delta_max_qbar_min_rad; at qbar_full_authority_pa it reaches the
    # full delta_max_rad from the rocket TOML.
    # Set qbar_full_authority_pa <= qbar_min_authority_pa to disable scheduling.
    qbar_min_authority_pa: float = 500.0
    qbar_full_authority_pa: float = 5000.0
    delta_max_qbar_min_rad: float = 0.05  # ~2.9 deg at low q-bar
    # Above this dynamic pressure the fin authority is reduced again to avoid
    # destructive post-burnout drag while the rocket is still very fast.
    qbar_high_authority_pa: float = 10000.0
    delta_max_qbar_high_rad: float = 0.10471975511965977  # 6 deg

    # Terminate-on-apogee tuning mode (Task 4 / User decision 4)
    # When True, the simulation ends at apogee (faster for tuning loops).
    # When False (default), the simulation continues through descent.
    terminate_on_apogee: bool = False

    # Paths
    reference_path: str = "data/trajectory/vertical.csv"
    results_dir: str = "results"

    # Launch site / Rail (English names)
    latitude: float = 42.3402247448
    longitude: float = -6.2713407985
    elevation_asl_m: float = 1000.0
    rail_length_m: float = 6.0
    heading_deg: float = 2.0
    inclination_deg: float = 87.0

    # Environment / Atmosphere
    launch_date: Tuple[int, int, int, int] = (
        2025,
        5,
        7,
        12,
    )  # (year, month, day, hour_UTC)
    atmosphere_type: str = (
        "auto"  # "auto" | "Reanalysis" | "Forecast" | "standard_atmosphere"
    )
    atmosphere_file: Optional[str] = (
        None  # None = auto-download; or path to local .nc / "GFS"
    )

    # Flags
    use_wind: bool = False  # True = real atmosphere (auto/Forecast/Reanalysis); False = standard_atmosphere
    save_results: bool = True
    show_plots: bool = True

    # Internal: populated by rocket_builder from TOML [control_actuation]
    # These are set at runtime so the controller can compute cD diagnostics
    # without importing the TOML directly.
    _delta_max_rad_from_toml: float = 0.3490658503988659
    _cN_delta_per_rad: float = 9.343586365106
    _cy_delta_per_rad: float = 9.343586365106
    _k_drag_induced: float = 0.295907824866


def load_config():
    """
    Returns execution parameters for the experiment.
    """
    return Config()
