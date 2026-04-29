# Input/Output Specifications

## Input Files

### 1. Rocket Definition: `data/rockets/*.toml`

TOML format defining rocket geometry, aerodynamics, and control actuation parameters.

**Example**: `data/rockets/leon_2.toml`

```toml
id = "leon_2"
name = "Leon 2"
description = "Cohete de referencia para la ruta oficial V1"

[geometry]
dry_mass_kg = 22.0
radius_m = 0.075
length_m = 2.8
center_of_mass_without_motor_m = 1.35
inertia_xx_kg_m2 = 0.12      # Roll inertia
inertia_yy_kg_m2 = 8.50      # Pitch inertia
inertia_zz_kg_m2 = 8.50      # Yaw inertia

[fins]
count = 4
configuration = "+"            # Cross configuration
root_chord_m = 0.24
tip_chord_m = 0.10
span_m = 0.12
sweep_length_m = 0.08
cant_angle_deg = 0.0
position_from_tail_m = 0.20

[stability_derivatives]
clalpha_per_rad = 4.8          # Lift slope
cmalpha_per_rad = -1.2          # Pitch moment slope
cybeta_per_rad = 0.0            # Side force slope
cnbeta_per_rad = 1.1            # Yaw moment slope

[control_actuation]
reference_area_m2 = 0.01767     # Fin reference area (m²)
reference_length_m = 0.15        # Fin reference length (m)
fin_aerodynamic_center_x_m = 0.7  # From nose tip (m)
fin_aerodynamic_center_y_m = 0.15 # Radial arm (m)
cN_delta_per_rad = 4.8           # Normal force derivative wrt delta
cm_delta_per_rad = -25.2         # Pitch moment derivative wrt delta
cy_delta_per_rad = 4.8           # Side force derivative wrt delta
cn_moment_delta_per_rad = -25.2  # Yaw moment derivative wrt delta
cl_delta_per_rad = 0.0           # Roll moment derivative wrt delta
cd_delta_per_rad = 0.0           # Drag delta derivative
k_drag_induced = 0.0            # Induced drag coefficient
delta_max_rad = 0.349            # Max fin deflection (~20°)
delta_dot_max_rad_s = 5.236      # Max deflection rate (rad/s)

[motor_interface]
mount_diameter_m = 0.075
mount_length_m = 0.80
max_motor_mass_kg = 12.0
recommended_motor_ids = ["pro75_3g"]

[drag]
default_drag_id = "leon_2_drag"
allowed_drag_ids = ["leon_2_drag", "leon_2_nominal"]
```

**Usage**: Loaded by `initial_data.py::load_initial_case_data()` using `toml.load()`.

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

**Usage**: Loaded by `src/rocket_builder.py::build_rocket()` using `pandas.read_csv()`, passed to `SolidMotor(thrust_source=...)`.

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

**Generate vertical reference**:
```bash
uv run python -c "from src.gen_reference import generate_vertical_reference; generate_vertical_reference('data/trajectory/vertical.csv', max_altitude=1000, duration=20)"
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
| `trajectory_3d.png` | 3D trajectory comparison | Full flight |
| `trajectory_2d_projections.png` | XY, XZ, YZ views | Full flight |
| `position_per_axis.png` | Per-axis position tracking | Control phase |
| `tracking_errors.png` | Tracking error norm and per-axis | Control phase |
| `fin_actuation.png` | Fin deflection history | Control phase |
| `attitude_euler.png` | Euler angles (roll, pitch, yaw) | Control phase |
| `body_rates.png` | Body angular rates | Control phase |

## Control Phase Detection

The "control phase" is identified by `src/utils.py::get_control_window_indices()`:

```python
# Control is active when ANY fin has deflection > 1e-6 rad
ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
start_idx = first timestep with active control
end_idx = apogee (max altitude)
```

**Control activation conditions** (in `src/controllers.py::fin_controller()`):
1. `t >= config.control_start_delay_s` (default: 3.0s)
2. `z_local >= config.control_start_min_height_above_launch_m` (default: 11.0m)
3. `vz > 0` (ascending phase)

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
