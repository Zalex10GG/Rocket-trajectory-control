# Input/Output Specifications

## Input Files

### 1. Rocket Definition: `data/rockets/*.toml`

TOML format defining rocket geometry, aerodynamics, and control actuation parameters.

**Example**: `data/rockets/leon_2.toml`

```toml
id = "leon_2"
name = "Leon 2"
description = "Cohete de referencia"

[nosecone]
kind = "vonkarman"
length_m = 0.5
base_radius_m = 0.05
position_m = 0

[body]
dry_mass_kg = 8.0
radius_m = 0.05
length_m = 2.556
center_of_mass_without_motor_m = 1.29
inertia_xx_kg_m2 = 0.08      # Roll inertia
inertia_yy_kg_m2 = 0.08      # Pitch inertia
inertia_zz_kg_m2 = 0.01      # Yaw inertia (note: 0.01 for Leon 2)
coordinate_system_orientation = "tail_to_nose"

[fins]
count = 4
root_chord_m = 0.203
tip_chord_m = 0.0625
span_m = 0.102
sweep_angle_deg = 36.0        # Using sweep_angle, not sweep_length
position_from_tail_m = 0.203
cant_angle_deg = 0.0
controlled = true

[control_actuation]
reference_area_m2 = 0.007853981633974483   # Fin reference area (m²)
reference_length_m = 0.1452                 # MAC (m)
cN_delta_per_rad = 9.343586365106          # Normal force derivative (increment only)
cy_delta_per_rad = 9.343586365106          # Side force derivative (increment only)
cl_delta_per_rad = 0.5                     # Roll moment derivative
k_drag_induced = 0.295907824866           # Induced drag factor
delta_max_rad = 0.3490658503988659        # Max fin deflection (~20°)
delta_dot_max_rad_s = 5.235987755982989    # Max deflection rate (rad/s)

[motor]
burn_time_start_s = 0.1
burn_time_end_s = 2.129
chamber_radius_m = 0.0375
chamber_height_m = 0.486
chamber_position_m = 0.0
propellant_initial_mass_kg = 1.755
nozzle_radius_m = 0.004
dry_mass_kg = 1.586
center_of_dry_mass_position_m = 0.25
dry_inertia_xx_kg_m2 = 0.001
dry_inertia_yy_kg_m2 = 0.001
dry_inertia_zz_kg_m2 = 0.0
nozzle_position_m = 0.0
coordinate_system_orientation = "nozzle_to_combustion_chamber"
```

**Usage**: 
- Loaded by `initial_data.py::load_initial_case_data(config)` using `toml.load()`
- `config` provides launch site parameters (latitude, longitude, elevation, etc.)
- Motor section uses `GenericMotor` (not `SolidMotor`)

### 2. Motor Thrust Curve: `data/motors/*.csv`

CSV format with motor thrust data.

**Columns**:
- `time_s`: Time from ignition (seconds)
- `thrust_N`: Thrust force (Newtons)

**Example**:
```csv
time_s,thrust_N
0.0,0.0
0.1,150.0
0.2,450.0
...
```

**Usage**: Loaded by `src/rocket_builder.py::build_rocket()` using `pandas.read_csv()`, passed to `GenericMotor(thrust_source=...)`. 

**Note**: Motor parameters (burn time, chamber dimensions, inertia) are now in `[motor]` section of the TOML file, not hardcoded.

### 3. Drag Coefficient: `data/drag/*.csv`

CSV format with drag coefficient vs Mach number.

**Columns**:
- `mach`: Mach number (dimensionless)
- `cd`: Drag coefficient (dimensionless)

**Example**:
```csv
mach,cd
0.01,0.434
0.02,0.508
0.03,0.525
...
```

**Usage**: Loaded by `initial_data.py` and passed to `Rocket(power_off_drag=..., power_on_drag=...)`.

### 4. Reference Trajectory: `data/trajectory/*.csv`

CSV format defining the desired trajectory to track.

**Columns**:
- `time_s`: Time from launch (seconds)
- `x_enu_m`, `y_enu_m`, `z_enu_m`: Position in **local ENU** coordinates (meters)
- `vx_enu_m_s`, `vy_enu_m_s`, `vz_enu_m_s`: Velocity in **local ENU** (m/s)

**Important**: All coordinates are relative to launch pad (0, 0, 0).

**Example** (vertical trajectory):
```csv
time_s,x_enu_m,y_enu_m,z_enu_m,vx_enu_m_s,vy_enu_m_s,vz_enu_m_s
0.0,0.0,0.0,0.0,0.0,0.0,100.0
0.01,0.0,0.0,1.0,0.0,0.0,100.0
0.02,0.0,0.0,2.0,0.0,0.0,100.0
...
```

**Usage**: 
- Loaded by `src/reference.py::load_reference_trajectory()`
- Creates `scipy.interpolate.interp1d` objects for each column
- Sampled at arbitrary times via `sample_reference(reference, time_s)`

**Provenance**: The default `data/trajectory/vertical.csv` is an **artificially generated vertical target** produced by `src/gen_reference.py`. It is NOT an uncontrolled passive baseline from the actual launch configuration. It defines a simple vertical ascent/descent profile with zero lateral displacement.

**Generate vertical reference**:
```bash
uv run py -c "from src.gen_reference import generate_vertical_reference; generate_vertical_reference('data/trajectory/vertical.csv', max_altitude=1000, duration=20)"
```

## Output Files

All outputs are saved to `results/<run_id>/` where `<run_id>` is a timestamp (`YYYYMMDD_HHMMSS`).

### 1. `flight_history.csv`

Complete simulation state history at each timestep.

**Columns**:
| Column | Description | Units |
|---------|-------------|-------|
| `time_s` | Simulation time | seconds |
| `x_local_m` | East position (local ENU) | meters |
| `y_local_m` | North position (local ENU) | meters |
| `z_local_m` | Up position (local ENU) | meters |
| `z_asl_m` | Absolute altitude ASL | meters |
| `vx` | East velocity | m/s |
| `vy` | North velocity | m/s |
| `vz` | Up velocity | m/s |
| `q0` | Quaternion scalar (w) | dimensionless |
| `q1` | Quaternion x component | dimensionless |
| `q2` | Quaternion y component | dimensionless |
| `q3` | Quaternion z component | dimensionless |
| `p` | Roll rate (body X) | rad/s |
| `q` | Pitch rate (body Y) | rad/s |
| `r` | Yaw rate (body Z) | rad/s |
| `delta1` | Fin 1 deflection (0°) | radians |
| `delta2` | Fin 2 deflection (90°) | radians |
| `delta3` | Fin 3 deflection (180°) | radians |
| `delta4` | Fin 4 deflection (270°) | radians |

**Quaternion format**: `[w, x, y, z]` representing ENU → Body rotation.

**Fin numbering**: Cross configuration (+), 0° = right (Fin 1).

### 2. `flight_summary.csv`

Key flight events and aggregate metrics (single-row CSV).

**Columns**:
| Column | Description | Units |
|---------|-------------|-------|
| `launch_altitude_asl_m` | Launch site elevation | meters |
| `max_altitude_asl_m` | Maximum altitude (ASL) | meters |
| `max_altitude_local_m` | Maximum altitude (local ENU) | meters |
| `time_of_apogee_s` | Time at apogee | seconds |
| `final_time_s` | Simulation end time | seconds |
| `max_speed_m_s` | Maximum velocity magnitude | m/s |
| `control_phase_start_s` | Control activation time | seconds |
| `control_phase_end_s` | Control deactivation time (apogee) | seconds |
| `control_phase_duration_s` | Control window duration | seconds |
| `max_fin_deflection_deg` | Maximum fin deflection | degrees |
| `fin_saturation_ratio` | Fraction of control samples at saturation | dimensionless (0-1) |

### 3. `metrics.json`

Detailed control performance metrics in JSON format.

**Structure**:
```json
{
    "ctrl_mae_3d_m": 5.23,           // Mean absolute 3D tracking error (control phase)
    "ctrl_rmse_3d_m": 6.81,          // Root mean square 3D error (control phase)
    "ctrl_max_error_3d_m": 12.45,     // Maximum 3D error (control phase)
    "ctrl_mae_x_m": 2.10,            // MAE in X (East)
    "ctrl_mae_y_m": 1.85,            // MAE in Y (North)
    "ctrl_mae_z_m": 4.92,            // MAE in Z (Up)
    "max_fin_deflection_deg": 12.5,   // Maximum fin deflection
    "fin_saturation_ratio": 0.05,      // Saturation ratio (0-1)
    "summary": { ... }                // Same as flight_summary.csv
}
```

**Note**: All `ctrl_*` metrics are computed only during the **control phase** (when fins are actively deflecting, from activation to apogee).

### 4. Plots: `plots/*.png`

Seven analysis plots (see [plots.md](plots.md) for details):

| File | Description | Scope |
|------|-------------|-------|
| `simulation/trajectory_3d.png` | 3D trajectory comparison | Full flight |
| `simulation/trajectory_2d_projections.png` | XY, XZ, YZ views | Full flight |
| `simulation/rocket.png` | Rocket diagram | Static |
| `simulation/static_margin.png` | Static margin | Static |
| `simulation/motor_thrust.png` | Motor thrust curve | Static |
| `control/position_per_axis.png` | Per-axis position tracking | Active control |
| `control/tracking_errors.png` | Tracking error norm and per-axis | Active control |
| `control/fin_actuation.png` | Fin deflection history | Active control |
| `control/attitude_euler.png` | Euler angles (roll, pitch, yaw) | Active control |
| `control/body_rates.png` | Body angular rates | Active control |
| `control/trajectory_3d.png` | 3D trajectory (control phase) | Active control |
| `control/trajectory_2d_projections.png` | 2D projections (control phase) | Active control |

### 5. `controller_diagnostics.csv`

Per-sample controller audit trail (new artifact).

**Columns**:
| Column | Description | Units |
|--------|-------------|-------|
| `time_s` | Callback timestamp | s |
| `control_active` | Whether control was active | bool |
| `cutoff_reason` | Why control was inactive | string |
| `q_dynamic_pa` | Dynamic pressure | Pa |
| `airspeed_m_s` | Airspeed | m/s |
| `delta_limit_rad` | Effective authority limit | rad |
| `effective_cD` | Control-induced drag coefficient | - |
| `raw_deltas_rad_0..3` | Raw deltas before limiting | rad |
| `limited_deltas_rad_0..3` | Final deltas after all limits | rad |
| `position_error_enu_m_0..2` | Position error vector | m |
| `velocity_error_enu_m_s_0..2` | Velocity error vector | m/s |
| `attitude_error_quat_0..3` | Attitude error quaternion | - |
| `commanded_accel_enu_m_s2_0..2` | Commanded acceleration | m/s² |

## Control Phase Detection

The "control phase" is identified by two functions in `src/utils.py`:

### Active-Control Window
```python
# Authoritative (from controller diagnostics):
active_times = [d["time_s"] for d in diag if d["control_active"]]
start = first active time
end = last active time

# Fallback: nonzero deltas
ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
start = first True index
end = last True index
```

### Ascent Window
```python
start = same as active-control start
end = apogee (max altitude in Z)
```

**Control activation conditions** (in `src/controllers.py::fin_controller()`):
1. `t >= config.control_start_delay_s` (default: 3.0s)
2. `z_local >= config.control_start_min_height_above_launch_m` (default: 11.0m)
3. `vz > 0` (ascending phase)
4. `q_dynamic >= q_min_cutoff_pa` (sufficient aerodynamic authority)
5. `t <= ref_time_limit` (within reference horizon)

## Data Flow Summary

```
Input Files                   Python Objects              Output Files
───────────                   ─────────────              ────────────
leon_2.toml  ──────────→   case_data["rocket_params"] 
cesaroni_...csv ─────────→   thrust_data (numpy array) 
leon_2_drag.csv ─────────→  drag_path (string)        
vertical.csv ────────────→   reference dict ──────────→ metrics.json
                                │                        flight_summary.csv
                                ↓
                          simulation.py
                                │
                                ↓
                          flight_history (list of dicts)
                                │
                                ├─→ flight_history.csv
                                └─→ plots/*.png
```
