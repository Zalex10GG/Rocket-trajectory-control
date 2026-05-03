# Rocket Control TFG - Sounding Rocket Trajectory Control

## Overview

This project implements a **6-DOF trajectory control system for sounding rockets** using rear-fin deflection. The simulation uses [RocketPy](https://github.com/RocketPy-Team/RocketPy) v1.12.1 for physics integration and leverages its private `_Controller` infrastructure for closed-loop fin control via `GenericSurface`.

**Current active path**: Rear-fin control with PID attitude control and PD guidance for trajectory tracking. AirBrakes are not used.

**Key features**:
- Uses `GenericMotor` (not `SolidMotor`) for motor definition from TOML parameters
- Control actuation limits (`delta_max_rad`, `delta_dot_max_rad_s`) defined in rocket TOML, not `config.py`
- `control_start_min_height_above_launch_m` is derived as `rail_length_m + safety_margin_m`
- New outputs: `manifest.json`, `effective_config.json`, `rocket_definition.toml`, `rocket_artifacts.json`

## Quick Start

### Prerequisites

- Python >= 3.12
- [uv](https://github.com/astral-sh/uv) package manager

### Install Dependencies

```bash
uv sync
```

### Run the Simulation

```bash
uv run main.py
```

Or using the installed entrypoint:

```bash
uv run rocket-control
```

This will:
1. Load the Leon 2 rocket configuration from `data/rockets/leon_2.toml`
2. Simulate a controlled flight tracking a vertical reference trajectory
3. Compute tracking metrics
4. Generate plots in `results/<run_id>/plots/`
5. Export flight history, metrics, and artifacts to `results/<run_id>/`

### Configuration

**Execution parameters** (`config.py`):
- **Control gains**: `Kp_guidance`, `Kd_guidance`, `Kp_attitude`, `Ki_attitude`, `Kd_attitude`, `Kp_roll`
- **Timing**: `control_dt_s` (50 Hz default), `control_start_delay_s`, `safety_margin_m`
- **Launch site**: `latitude`, `longitude`, `elevation_asl_m`, `rail_length_m`, `heading_deg`, `inclination_deg`
- **Flags**: `save_results`, `show_plots`
- **Paths**: `reference_path`, `results_dir`

**Actuation limits** (in `data/rockets/leon_2.toml` under `[control_actuation]`):
- `delta_max_rad`: Max fin deflection (default: ~20°)
- `delta_dot_max_rad_s`: Max deflection rate

**Note**: `control_start_min_height_above_launch_m` is derived as `rail_length_m + safety_margin_m` in `rocket_builder.py`.

## Coordinate Convention

All trajectories (real and reference) use **local ENU (East-North-Up)** coordinates with:
- **Origin**: Launch pad position (0, 0, 0)
- **X**: East
- **Y**: North
- **Z**: Up

RocketPy internal integration uses absolute ASL positions, but the controller and all outputs are converted to local ENU relative to the launch position.

**Attitude representation**: Quaternions in `[w, x, y, z]` format (ENU → Body frame). Euler angles (roll, pitch, yaw) use ZYX convention.

## Project Structure

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
        └── plots/                  # 7 analysis plots
```

## Input Files

### Rocket Definition (`data/rockets/leon_2.toml`)
Defines:
- `[nosecone]`: Nose cone geometry and position
- `[body]`: Mass, dimensions, inertia (note: `inertia_zz_kg_m2 = 0.01` for Leon 2)
- `[fins]`: Passive stabilization fins (4 trapezoidal fins) with `sweep_angle_deg`
- `[control_actuation]`: Rear fin control parameters (reference area, coefficients, delta limits)
- `[motor]`: Motor parameters for `GenericMotor` (burn time, chamber dimensions, inertia)

### Motor Thrust Curve (`data/motors/*.csv`)
CSV with `time_s` and `thrust_N` columns.

### Drag Coefficient (`data/drag/*.csv`)
CSV with `mach` and `cd` columns.

### Reference Trajectory (`data/trajectory/*.csv`)
CSV with columns:
- `time_s`: Time in seconds
- `x_enu_m`, `y_enu_m`, `z_enu_m`: Position in local ENU (m)
- `vx_enu_m_s`, `vy_enu_m_s`, `vz_enu_m_s`: Velocity in local ENU (m/s)

Generate a vertical reference:
```bash
uv run py -c "from src.gen_reference import generate_vertical_reference; generate_vertical_reference('data/trajectory/vertical.csv', max_altitude=1000, duration=20)"
```

## Output Files

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
7 analysis plots (see [docs/plots.md](docs/plots.md) for details):
1. `trajectory_3d.png` - 3D trajectory comparison (full flight)
2. `trajectory_2d_projections.png` - XY, XZ, YZ projections (full flight)
3. `position_per_axis.png` - Per-axis position tracking (control phase)
4. `tracking_errors.png` - Tracking error norm and per-axis (control phase)
5. `fin_actuation.png` - Fin deflection history (control phase)
6. `attitude_euler.png` - Euler angles (control phase)
7. `body_rates.png` - Body angular rates (control phase)

## Control Architecture

```
Reference Trajectory (ENU)
        │
        ▼
   [PD Guidance] ← Desired acceleration in ENU (with gravity compensation from Environment)
        │
        ▼
   [Quaternion Attitude Control] ← Desired quaternion (ENU → Body)
        │
        ▼
   [PID Attitude Error] ← Error quaternion (Body frame)
        │
        ├─ Pitch/Yaw: PID on q_error[y], q_error[z]
        └─ Roll: P on error[0] - Kd_attitude * body rate p
        │
        ▼
   [Mixer] ← Maps (pitch, yaw, roll) to 4 fin deflections
        │
        ▼
   [FinAdapter] ← Maps deltas to GenericSurface coefficients (cL, cQ, cD, cm=0, cn=0, cl)
        │
        ▼
   [RocketPy GenericSurface] ← Applies aerodynamic forces/moments (moments via CP-to-CG arm)
```

**Control activation**: Starts after `control_start_delay_s` (3s) AND above `control_start_min_height_above_launch_m` (derived: `rail_length_m + safety_margin_m`) AND while `vz > 0` (ascending) AND `t <= ref_time_limit` AND `vel_ref[2] > 0`.

**Fins configuration**: Cross (+) with 4 fins:
- Fin 1 (0°): Right
- Fin 2 (90°): Top
- Fin 3 (180°): Left
- Fin 4 (270°): Bottom

**Mixing law**:
```
d1 = u_yaw + u_roll
d2 = u_pitch + u_roll
d3 = -u_yaw + u_roll
d4 = -u_pitch + u_roll
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

## Documentation

Detailed documentation is in the `docs/` directory:

### Getting Started
- [Running the Simulation](docs/running_the_simulation.md) - Detailed execution guide
- [Input/Output Specifications](docs/io_specs.md) - File formats and data structures

### Technical Reference
- [Architecture](docs/architecture.md) - System design and data flow
- [Coordinate Systems](docs/coordinate_systems.md) - ENU convention details
- [Plots and Analysis](docs/plots.md) - Plot descriptions and interpretation

### Module Documentation (`docs/modules/`)
- [controllers.py](docs/modules/controllers.md) - Fin controller with PID and quaternion attitude
- [simulation.py](docs/modules/simulation.md) - RocketPy integration and flight execution
- [rocket_builder.py](docs/modules/rocket_builder.md) - Rocket and motor construction
- [fin_model.py](docs/modules/fin_model.md) - FinAdapter for GenericSurface
- [reference.py](docs/modules/reference.md) - Reference trajectory loading
- [metrics.py](docs/modules/metrics.md) - Tracking performance metrics
- [utils.py](docs/modules/utils.md) - Quaternion math and utilities
- [environment_builder.py](docs/modules/environment_builder.md) - RocketPy Environment setup

## Author

Alejandro Gil Getino - Universidad de León, 2026
