# Plots and Analysis

## Nominal Simulation Plots

Nominal simulation plots are written inside each run directory:

- `results/<run_id>/simulation/`
- `results/<run_id>/control/`

`simulation/` contains full-flight plots. `control/` contains active-control-window plots.

## `simulation/`

- `trajectory_3d.png`: full simulated trajectory and full reference trajectory in local ENU.
- `trajectory_2d_projections.png`: XY, XZ, and YZ projections.
- `velocity_per_axis.png`: simulated ENU velocity components over the full simulated flight.
- `attitude_euler.png`: full-flight Euler angles derived from the attitude quaternion.
- `cd_vs_mach.png`: configured base drag coefficient over the achieved Mach range.
- `motor_thrust.png`: RocketPy motor thrust plot.
- `static_margin.png`: RocketPy static margin plot.
- `rocket.png`: RocketPy rocket drawing.

## `control/`

- `position_per_axis.png`: simulated and reference ENU position components in the active control phase.
- `tracking_errors.png`: 3D tracking error and per-axis position errors in the active control phase.
- `fin_actuation.png`: four fin deflections and the dynamic q-bar authority limit.
- `attitude_euler.png`: control-phase Euler angles.
- `body_rates.png`: body angular rates.
- `velocity_per_axis.png`: simulated and reference ENU velocity components.
- `trajectory_3d.png`: control-phase 3D trajectory and matched reference.
- `trajectory_2d_projections.png`: control-phase XY, XZ, and YZ projections.
- `gain_evolution.png`: dynamic pressure, q-scale, and active proportional gains.
- `guidance_sources.png`: guidance and attitude source diagnostics from controller logs.

## Gain Sweep Plots

`tools/sweep_gain_scale.py` writes plots to `tools/results/sweep/`.

Top-level sweep plots:

- `sweep_error_summary.png`: 2x3 summary of main error metrics.
- `apogee_altitude_vs_gain_scale.png`: simulated apogee altitude versus gain scale, with the reference apogee altitude as a dashed constant line.
- `all_metrics_vs_gain_scale.png`: 12-subplot view of all plotted sweep metrics.

Per-metric plots are written to `tools/results/sweep/metrics/`:

- `kp_attitude.png`
- `ki_attitude.png`
- `kd_attitude.png`
- `mean_3d_error_m.png`
- `max_3d_error_m.png`
- `max_lateral_error_m.png`
- `apogee_3d_error_m.png`
- `apogee_lateral_error_m.png`
- `apogee_height_error_m.png`
- `apogee_altitude_real_m.png`
- `apogee_time_s.png`
- `max_fin_deflection_deg.png`

Plot titles use readable metric names without units. Axis labels include units where applicable.
