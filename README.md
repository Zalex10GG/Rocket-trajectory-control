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

`tools/sweep_gain_scale.py` writes:

- `tools/results/sweep/sweep_metrics.csv`
- `tools/results/sweep/sweep_error_summary.png`
- `tools/results/sweep/apogee_altitude_vs_gain_scale.png`
- `tools/results/sweep/all_metrics_vs_gain_scale.png`
- `tools/results/sweep/metrics/*.png`

The sweep CSV includes:

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

The controller activates after the configured delay and minimum height while the rocket and reference are still ascending. The simulation can be terminated at apogee by setting `config.terminate_on_apogee = True`.

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
