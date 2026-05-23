# Rocket Control TFG - Sounding Rocket Trajectory Control

## Overview

This project implements a 6-DOF trajectory control simulation for a sounding rocket using rear-fin deflection. The flight physics are integrated with [RocketPy](https://github.com/RocketPy-Team/RocketPy) v1.12.1, using RocketPy's private `_Controller` infrastructure to run a closed-loop fin controller during integration.

The active control architecture is rear-fin control through a RocketPy `GenericSurface` plus `FinAdapter`. Air brakes are not part of the control path.

## Quick Start

### Prerequisites

- Python 3.12 or newer
- [uv](https://github.com/astral-sh/uv)

### Install Dependencies

```bash
uv sync
```

### Run the Nominal Simulation

```bash
uv run main.py
```

Or with the installed entrypoint:

```bash
uv run rocket-control
```

The nominal run:

1. Loads `config.py`.
2. Loads the case data from `initial_data.py`.
3. Loads the reference trajectory from `config.reference_path`.
4. Builds the controller, environment, and RocketPy rocket.
5. Runs `src.simulation.simulate_controlled_flight`.
6. Computes metrics.
7. Exports CSV, JSON, diagnostics, and plots to `results/<run_id>/`.

### Run the Gain Scale Sweep

```bash
uv run py tools/sweep_gain_scale.py
```

The sweep changes `config.attitude_gain_scale`, runs one simulation per factor up to apogee, and writes the CSV and plots to `tools/results/sweep/`. The sweep interval is configured at the top of `tools/sweep_gain_scale.py` with:

- `SCALE_MIN`
- `SCALE_MAX`
- `SCALE_STEP`

## Configuration

All execution parameters are in `config.py`.

### Paths

- `rocket_path`: rocket TOML definition.
- `motor_path`: motor thrust curve CSV.
- `drag_path`: drag coefficient CSV.
- `reference_path`: reference trajectory CSV.
- `results_dir`: nominal simulation output directory.

### Guidance And Attitude Gains

- `Kp_direction_guidance`, `Kd_direction_guidance`: outer-loop position/velocity guidance gains.
- `Kp_attitude_zn`, `Ki_attitude_zn`, `Kd_attitude_zn`: pitch/yaw baseline attitude gains.
- `attitude_gain_scale`: multiplier applied to the three pitch/yaw baseline gains.
- `Kp_attitude`, `Ki_attitude`, `Kd_attitude`: calculated properties equal to baseline gain times `attitude_gain_scale`.
- `Kp_roll`, `Ki_roll`, `Kd_roll`: roll PID gains.

### Scheduling, Activation, And Limits

- `enable_gain_scheduling`: enables inverse dynamic-pressure gain scaling.
- `qbar_ref_pa`, `gain_scheduling_max_scale`, `q_min_cutoff_pa`: dynamic-pressure gain schedule parameters.
- `max_attitude_correction_deg`, `max_commanded_aoa_deg`: guidance command limits.
- `control_start_delay_s`, `safety_margin_m`: control activation parameters.
- `control_start_min_height_above_launch_m`: derived in `src.rocket_builder.build_rocket` as `rail_length_m + safety_margin_m`.
- `apogee_control_cutoff_delay_s`: cutoff margin near reference apogee.
- `actuator_command_filter_tau_s`: optional first-order smoothing of fin commands.
- `qbar_min_authority_pa`, `qbar_full_authority_pa`, `qbar_high_authority_pa`, `delta_max_qbar_min_rad`, `delta_max_qbar_high_rad`: dynamic-pressure deflection authority schedule.
- `terminate_on_apogee`: passed to RocketPy `Flight`.

### Environment

- `use_wind = False`: uses RocketPy `standard_atmosphere`.
- `use_wind = True`: uses `atmosphere_type`.
- `atmosphere_type = "auto"`: chooses `Reanalysis` for past launch dates and `Forecast` for future launch dates.
- `atmosphere_file`: optional local atmospheric file.

### Rocket TOML Actuation

The rocket TOML `[control_actuation]` section defines the control-surface physical limits and aerodynamic coefficients:

- `delta_max_rad`
- `delta_dot_max_rad_s`
- `reference_area_m2`
- `reference_length_m`
- `cN_delta_per_rad`
- `cy_delta_per_rad`
- `cl_delta_per_rad`
- `k_drag_induced`

## Coordinate Convention

Simulation outputs, references, metrics, and plots use local ENU coordinates:

- X: East
- Y: North
- Z: Up
- Origin: launch pad

RocketPy integrates positions in absolute coordinates including launch elevation. `src.simulation.simulate_controlled_flight` subtracts the initial launch position so controller tracking and exported local positions use `(0, 0, 0)` at the pad.

Attitude quaternions use scalar-first `[w, x, y, z]` format for ENU-to-body orientation.

## Project Structure

<<<<<<< HEAD
```
├── main.py                    # Entry point
├── config.py                  # Execution parameters
├── initial_data.py            # Loads rocket/motor/drag definitions from config
├── src/
│   ├── simulation.py         # RocketPy Flight integration and history extraction
│   ├── controllers.py        # PID fin controller with quaternion attitude control
│   ├── fin_model.py          # FinAdapter: maps deltas to GenericSurface coefficients
│   ├── rocket_builder.py     # Constructs RocketPy Rocket and GenericMotor
│   ├── environment_builder.py # Constructs RocketPy Environment
│   ├── reference.py          # Reference trajectory loading and sampling
│   ├── metrics.py            # Tracking performance metrics
│   ├── plots.py              # Plotting suite (full-flight + control-phase)
│   ├── utils.py              # Quaternion math and control window detection
│   ├── constants.py          # Shared constants (CONTROL_SURFACE_NAME)
│   └── gen_reference.py      # Utility to generate vertical reference trajectories
├── data/
│   ├── rockets/leon_2.toml  # Rocket geometry, motor, and control actuation params
│   ├── motors/*.csv          # Motor thrust curves
│   ├── drag/*.csv            # Drag coefficient vs Mach number
│   └── trajectory/*.csv      # Reference trajectory (time, pos, vel in ENU)
└── results/
    └── <run_id>/             # Output directory (timestamped with microseconds)
        ├── flight_history.csv
        ├── flight_summary.csv
        ├── metrics.json
        ├── manifest.json           # Git metadata, file hashes
        ├── effective_config.json   # Serializable config
        ├── rocket_definition.toml  # Copy of rocket TOML
        ├── rocket_artifacts.json   # Rocket stats
        └── plots/                  # 10 analysis plots (organized in simulation and control subdirectories)
=======
```text
main.py                       Nominal simulation entry point
config.py                     Execution parameters and calculated gains
initial_data.py               Loads configured rocket, motor, drag, and reference paths
src/
  controllers.py              Guidance, attitude PID, mixer, limits, diagnostics
  environment_builder.py      RocketPy Environment construction
  fin_model.py                FinAdapter GenericSurface coefficient bridge
  gen_reference.py            Reference generation helper
  metrics.py                  Tracking and control metrics
  plots.py                    Nominal simulation plot generation
  reference.py                Reference loading and interpolation
  rocket_builder.py           RocketPy Rocket, motor, surfaces, parachute
  simulation.py               RocketPy Flight execution and result export
  utils.py                    Quaternion math and control-window helpers
tools/
  sweep_gain_scale.py         Attitude gain-scale sweep up to apogee
  trajectory-creator.py       Passive trajectory reference generation
  tunning.py                  Pitch/roll identification and tuning support
data/
  rockets/leon_2.toml
  motors/cesaroni_pro75_3g_3727l1050.csv
  drag/leon_2_drag.csv
  trajectory/85degree.csv
results/
  <run_id>/                   Nominal simulation outputs
tools/results/sweep/          Gain sweep outputs
>>>>>>> alejandro
```

## Nominal Outputs

Each nominal run creates `results/<YYYYMMDD_HHMMSS>/` containing:

- `flight_history.csv`
- `flight_summary.csv`
- `metrics.json`
- `controller_diagnostics.csv`
- `effective_config.json`
- `rocket_definition.toml`
- `rocket_artifacts.json`
- `simulation/`
- `control/`

`simulation/` contains full-flight plots. `control/` contains plots focused on the active control phase.

## Gain Sweep Outputs

<<<<<<< HEAD
## Auxiliary Tools

The project includes auxiliary tools in the `tools/` directory to streamline trajectory generation, aerodynamic parameter estimation, and closed-loop controller tuning:

### 1. Reference Trajectory Creator (`tools/trajectory-creator.py`)
Generates a nominal 3D spatial reference trajectory in the local tangent ENU frame.
- **Run Command**:
  ```bash
  uv run py tools/trajectory-creator.py
  ```
- **Notes**: Generates an ideal trajectory under no wind and no noise conditions. The generated file is saved to `data/trajectory/vertical.csv` by default.

### 2. Aerodynamic Coefficient Calculator (`tools/calculate_control_coefficients.py`)
Estimates lifting and induced drag aerodynamic force derivatives ($C_{N_\delta}$, $C_{y_\delta}$, $k$) based on the rocket body and tail fin geometries defined in the rocket TOML file.
- **Run Command**:
  ```bash
  uv run py tools/calculate_control_coefficients.py
  ```
- **Notes**: Uses the subsonic Diederich lift-slope formulation with body-on-fin interference factors ($K_{TB}$). Outputs should be updated in the `[control_actuation]` section of the rocket TOML to align GenericSurface aerodynamics with geometry.

### 3. Ziegler-Nichols Controller Auto-Tuning (`tools/tunning.py`)
Excites the rocket via open-loop pulse and step fin commands at maximum dynamic pressure (max-Q, $t \approx 2.13$ s) to isolate and identify short-period system dynamics.
- **Run Command**:
  ```bash
  uv run py tools/tunning.py
  ```
- **Identified Models**:
  - **Pitch Loop**: Identifies a second-order underdamped transfer function relating pitch attitude ($\theta$) and rate ($\omega_x$) to fin commands:
    $$G_{pitch}(s) = \frac{\Theta(s)}{\Delta(s)} = \frac{K_p \omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$
  - **Roll Loop**: Identifies a first-order rate lag model:
    $$G_{roll}(s) = \frac{\Omega_z(s)}{\Delta_r(s)} = \frac{K_r}{\tau s + 1}$$
- **Suggested Gains**: Prints recommended base PID gains derived using Ziegler-Nichols transient response methods.
- **Diagnostics**: Generates time response fits, Bode plots, and pole-zero maps (saved in `tools/results/`):
  - `pitch_modes.png`: Precise pole locations on an aspect-ratio-corrected 1:1 map with constant damping lines ($\zeta$) and natural frequency circles ($\omega_n$).
  - `roll_modes.png`: Features the high-speed boundary layer roll damping pole at $s = -1/\tau \approx -200$ rad/s and the attitude integrator pole at $s = 0$.
=======
`tools/sweep_gain_scale.py` writes:

- `tools/results/sweep/sweep_metrics.csv`
- `tools/results/sweep/sweep_error_summary.png`
- `tools/results/sweep/apogee_altitude_vs_gain_scale.png`
- `tools/results/sweep/all_metrics_vs_gain_scale.png`
- `tools/results/sweep/metrics/*.png`
>>>>>>> alejandro

The sweep CSV includes:

<<<<<<< HEAD
Each run creates `results/<YYYYMMDD_HHMMSS>/` with:

### `flight_history.csv`
Complete simulation state at each timestep:
- `time_s`, `x_local_m`, `y_local_m`, `z_local_m`: Local ENU position
- `z_asl_m`: Absolute altitude ASL
- `vx`, `vy`, `vz`: Velocity in ENU
- `q0`, `q1`, `q2`, `q3`: Attitude quaternion (ENU → Body)
- `p`, `q`, `r`: Body angular rates (rad/s)
- `delta1`-`delta4`: Fin deflections (rad) for the 4 fins (cross configuration)

### `flight_summary.csv`
Key flight events:
- Launch altitude ASL, max altitude (ASL and local)
- Time of apogee, final time
- Max speed, control phase start/end/duration
- Max fin deflection, saturation ratio

### `metrics.json`
Control performance metrics (control phase only):
- `ctrl_mae_3d_m`: Mean absolute 3D tracking error
- `ctrl_rmse_3d_m`: Root mean square 3D error
- `ctrl_max_error_3d_m`: Maximum 3D error
- `ctrl_mae_x_m`, `ctrl_mae_y_m`, `ctrl_mae_z_m`: Per-axis MAE
- `max_fin_deflection_deg`, `fin_saturation_ratio`

### `plots/` Directory
10 analysis plots (see [docs/plots.md](docs/plots.md) for details):
1. `trajectory_3d.png` - 3D trajectory comparison (full flight and control phase)
2. `trajectory_2d_projections.png` - XY, XZ, and YZ projections (full flight and control phase)
3. `position_per_axis.png` - Per-axis position tracking with optimized legend placement (control phase)
4. `tracking_errors.png` - Tracking error norm and per-axis tracking deviations (control phase)
5. `fin_actuation.png` - Fin deflection history and dynamic deflection limits (control phase)
6. `attitude_euler.png` - Achievement versus command of roll, pitch, and yaw Euler angles (control phase)
7. `body_rates.png` - Angular rates ($\omega_x, \omega_y, \omega_z$) for damping and vibration analysis (control phase)
8. `velocity_per_axis.png` - Linear velocity tracking per axis (full flight and control phase)
9. `cd_vs_mach.png` - Aerodynamic drag coefficient versus Mach number (full flight)
10. `gain_evolution.png` - Gain scheduling dynamics showing pressure ($q$), scaling factor ($q_{scale}$), and active scheduled gains ($K_{p,attitude}$, $K_{p,roll}$) (control phase)
=======
- gain scale and active attitude gains
- mean 3D error up to apogee
- max 3D error up to apogee
- max lateral error up to apogee
- 3D error at apogee
- lateral error at apogee
- maximum-height error at apogee
- simulated apogee altitude
- reference apogee altitude
- reference altitude at simulated apogee time
- apogee time
- maximum fin deflection
>>>>>>> alejandro

## Control Architecture

```text
Reference trajectory (local ENU)
        |
        v
PD direction guidance
        |
        v
Commanded nose direction with angle-of-attack limit
        |
        v
Desired attitude quaternion (ENU -> Body)
        |
        v
Pitch/yaw/roll PID with dynamic-pressure gain scheduling
        |
        v
Mixer: pitch/yaw/roll commands -> 4 fin deflections
        |
        v
Rate limit, q-bar authority limit, saturation
        |
        v
FinAdapter reads controller["current_deltas"]
        |
        v
RocketPy GenericSurface coefficients
```

<<<<<<< HEAD
**Control activation**: Starts after `control_start_delay_s` (3s) AND above `control_start_min_height_above_launch_m` (derived: `rail_length_m + safety_margin_m`) AND while `vz > 0` (ascending) AND `t <= ref_time_limit` AND `vel_ref[2] > 0`.

**Fins configuration**: Cross (+) with 4 fins:
- Fin 1 (0°): Right
- Fin 2 (90°): Top
- Fin 3 (180°): Left
- Fin 4 (270°): Bottom

**Mixing law**:
```
d1 = -u_pitch + u_roll
d2 = -u_yaw + u_roll
d3 = u_pitch + u_roll
d4 = u_yaw + u_roll
```

**Rate limiting**: `delta_dot_max_rad_s` from TOML is enforced before saturation.

## Known Limitations

1. **RocketPy Private API**: Uses `_Controller` (private infrastructure). Future RocketPy versions may break compatibility. See `src/simulation.py` docstring for details.

2. **Controller Sampling Mismatch**: The controller callback is sampled by RocketPy's ODE solver, which may not align exactly with `control_dt_s`. Nearest-neighbor lookup is used to reconcile timestamps.

3. **Simplified Guidance**: PD guidance with gravity compensation (gravity obtained from RocketPy `Environment`). No optimal trajectory generation or feedforward terms.

4. **No Wind/Gust Modeling**: Environment uses standard ISA atmosphere with no wind disturbances.

5. **Quaternion Convention**: Ensure consistency between ENU→Body quaternion format (`[w, x, y, z]`) and RocketPy's internal representation.

6. **Fin Rate Limiting**: Rate limiting (`delta_dot_max_rad_s`) is implemented in `controllers.py`, not in `FinAdapter`.

7. **Single Reference**: Currently only vertical trajectory reference. Lateral trajectory tracking not validated.

8. **No EKF/State Estimation**: Uses perfect state feedback from simulation. Real-world implementation would require sensor fusion.

9. **Moment Derivatives**: `cm_delta` and `cn_delta` return 0.0 in `FinAdapter`. Moments are computed by RocketPy using CP-to-CG arm.
=======
The controller activates after the configured delay and minimum height while the rocket and reference are still ascending. The simulation can be terminated at apogee by setting `config.terminate_on_apogee = True`.
>>>>>>> alejandro

## Documentation

- [Running the Simulation](docs/running_the_simulation.md)
- [Input and Output Specifications](docs/io_specs.md)
- [Architecture](docs/architecture.md)
- [Coordinate Systems](docs/coordinate_systems.md)
- [Plots and Analysis](docs/plots.md)
- [Gain Scale Sweep](docs/tools/sweep_gain_scale.md)
- [controllers.py](docs/modules/controllers.md)
- [environment_builder.py](docs/modules/environment_builder.md)
- [fin_model.py](docs/modules/fin_model.md)
- [metrics.py](docs/modules/metrics.md)
- [reference.py](docs/modules/reference.md)
- [rocket_builder.py](docs/modules/rocket_builder.md)
- [simulation.py](docs/modules/simulation.md)
- [utils.py](docs/modules/utils.md)

## Author

Alejandro Gil Getino - Universidad de Leon, 2026
