# Input and Output Specifications

## Nominal Inputs

The nominal case is configured through `config.py` and `initial_data.py`.

Default paths:

- Rocket definition: `data/rockets/leon_2.toml`
- Motor thrust curve: `data/motors/cesaroni_pro75_3g_3727l1050.csv`
- Drag curve: `data/drag/leon_2_drag.csv`
- Reference trajectory: `data/trajectory/85degree.csv`

## Rocket TOML

`data/rockets/leon_2.toml` defines:

- `nosecone`: nose geometry.
- `body`: dry mass, radius, inertia, center of mass, coordinate orientation.
- `fins`: passive trapezoidal fins.
- `control_actuation`: active control-surface parameters and limits.
- `motor`: `GenericMotor` parameters.
- `parachute`: optional parachute configuration.

## Motor CSV

The motor CSV is read with `pandas.read_csv(..., comment="#")` and passed to RocketPy `GenericMotor` as a thrust source.

## Drag CSV

The drag CSV is passed to RocketPy as both `power_off_drag` and `power_on_drag`.

## Reference CSV

Reference trajectories contain:

- `time_s`
- `x_enu_m`, `y_enu_m`, `z_enu_m`
- `vx_enu_m_s`, `vy_enu_m_s`, `vz_enu_m_s`

`src.reference.load_reference_trajectory()` creates interpolators for every CSV column and stores `peak_z_enu` when `z_enu_m` is present.

## Nominal Outputs

Each nominal simulation writes to `results/<run_id>/`.

### `flight_history.csv`

Contains one row per extracted RocketPy solution sample:

- `time_s`
- `x_local_m`, `y_local_m`, `z_local_m`
- `z_asl_m`
- `vx`, `vy`, `vz`
- `q0`, `q1`, `q2`, `q3`
- `p`, `q`, `r`
- `delta1`, `delta2`, `delta3`, `delta4`
- `qref0`, `qref1`, `qref2`, `qref3`
- `q_dynamic_pa`
- `mach`

### `flight_summary.csv`

Contains one row with:

- launch altitude ASL
- maximum altitude ASL and local
- time of apogee
- final simulation time
- maximum speed
- active-control timing
- ascent-window timing
- maximum fin deflection
- saturation ratio
- lateral RMSE
- control drag diagnostics
- commanded angle-of-attack diagnostics
- duplicate callback count

### `metrics.json`

Contains tracking metrics, saturation metrics, diagnostics, and the `summary` object exported to `flight_summary.csv`.

### `controller_diagnostics.csv`

Contains active controller callback diagnostics, including:

- control activity and cutoff state
- dynamic pressure and airspeed
- current deflection limit
- raw and limited fin commands
- position and velocity errors
- attitude error quaternion
- effective control drag

### Rocket and Config Artifacts

- `effective_config.json`
- `rocket_definition.toml`
- `rocket_artifacts.json`

## Gain Sweep Outputs

`tools/sweep_gain_scale.py` writes to `tools/results/sweep/`.

### `sweep_metrics.csv`

Columns:

- `gain_scale`
- `kp_attitude`, `ki_attitude`, `kd_attitude`
- `mean_3d_error_m`
- `max_3d_error_m`
- `max_lateral_error_m`
- `apogee_3d_error_m`
- `apogee_lateral_error_m`
- `apogee_height_error_m`
- `apogee_altitude_real_m`
- `reference_max_altitude_m`
- `reference_altitude_at_apogee_time_m`
- `apogee_time_s`
- `max_fin_deflection_deg`

### Sweep Figures

See [Plots and Analysis](plots.md).
