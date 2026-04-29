# Architecture Documentation

## System Overview

The Rocket Control TFG project implements a closed-loop trajectory control system for sounding rockets using rear-fin deflection. The system integrates with RocketPy's 6-DOF flight simulation through its private `_Controller` infrastructure.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        main.py                                 │
│  Entry point: orchestrates loading, simulation, analysis      │
└────────────────────┬──────────────────────────────────────────┘
                     │
        ┌────────────┼────────────────┐
        ▼            ▼                 ▼
┌──────────────┐ ┌──────────┐ ┌──────────────────┐
│ config.py    │ │initial_  │ │ src/              │
│              │ │data.py   │ │                  │
│ Parameters   │ │          │ │ simulation.py     │
│ for control, │ │ Loads    │ │  - Flight sim    │
│ timing,      │ │ rocket,  │ │  - _Controller   │
│ launch site  │ │ motor,   │ │  - History extract│
└──────────────┘ │ drag     │ └──────────────────┘
                 └──────────┘           │
                                        ▼
                   ┌────────────────────────────────────┐
                   │     RocketPy Flight Object         │
                   │  - 6-DOF Equations of Motion      │
                   │  - Variable mass (motor burn)      │
                   │  - Aerodynamics (drag, fins)       │
                   │  - Environment (ISA atmosphere)     │
                   └────────────────────────────────────┘
```

## Data Flow

### 1. Initialization Phase

```
config.py::load_config()
    ↓
Returns: Config object with timing, gains, launch site, paths

initial_data.py::load_initial_case_data()
    ↓
Reads: data/rockets/leon_2.toml
    ↓
Returns: dict with rocket_params, motor_path, drag_path, trajectory_path

src/environment_builder.py::build_environment()
    ↓
Creates: RocketPy Environment (latitude, longitude, elevation, ISA atmosphere)

src/rocket_builder.py::build_rocket()
    ↓
Creates: RocketPy Rocket + SolidMotor
    ↓
Attaches: FinAdapter → GenericSurface (rear fins control)
    ↓
Adds: Nose cone, trapezoidal fins (passive), parachute

src/controllers.py::build_controller()
    ↓
Returns: Controller state dict (integral_error, previous_error, deltas_history)
```

### 2. Simulation Phase

```
src/simulation.py::simulate_controlled_flight()
    │
    ├─ Creates controller_callback(t, sampling_rate, state, ...)
    │   State format: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
    │
    ├─ Creates RocketPy _Controller with:
    │   - interactive_objects: [GenericSurface]
    │   - controller_function: controller_callback
    │   - sampling_rate: 1/control_dt_s
    │
    ├─ rocket._add_controllers(ctrl_obj)
    │
    └─ Flight(rocket, environment, ...)
        │
        └─ During integration, at each callback:
            │
            └─ src/controllers.py::fin_controller(t, state, controller, config, reference)
                │
                ├─ Check control activation (time, height, vz > 0)
                │
                ├─ Sample reference trajectory at time t
                │   Returns: position_enu_m, velocity_enu_m_s
                │
                ├─ PD Guidance:
                │   accel_cmd = Kp*(pos_ref - pos) + Kd*(vel_ref - vel) + [0,0,9.81]
                │
                ├─ Compute desired quaternion (ENU→Body):
                │   Align rocket nose (Body Z) with accel_cmd direction
                │
                ├─ Attitude PID in Body frame:
                │   q_error = q_ref * conj(q_real)
                │   e_roll = q_error[1], e_pitch = q_error[2], e_yaw = q_error[3]
                │   PID output: u_pitch, u_yaw
                │
                ├─ Roll damping: u_roll = -Kp_roll * w_body[0]
                │
                ├─ Mixer (4 fins, cross config):
                │   d1 = u_yaw + u_roll    (Fin 1: 0°, right)
                │   d2 = u_pitch + u_roll  (Fin 2: 90°, top)
                │   d3 = -u_yaw + u_roll   (Fin 3: 180°, left)
                │   d4 = -u_pitch + u_roll (Fin 4: 270°, bottom)
                │
                ├─ Saturate: clip to [-delta_max_rad, +delta_max_rad]
                │
                └─ Update controller["current_deltas"] and controller["deltas_history"][t]
                    ↓
                FinAdapter.get_current_deltas() reads controller["current_deltas"]
                    ↓
                GenericSurface coefficients updated:
                    cL = -cN_delta * (d2 - d4)/2
                    cQ = cy_delta * (d1 - d3)/2
                    cm = cm_delta * (d2 - d4)/2
                    cn = cn_delta * (d1 - d3)/2
                    cl = cl_delta * mean(deltas)
```

### 3. Post-Processing Phase

```
simulation.py extracts flight history:
    for each timestep in flight.solution:
        - pos_enu_m = solution[1:4] - launch_position (convert to local ENU)
        - vel_enu_m_s = solution[3:6]
        - attitude_quaternion = solution[6:10]
        - body_rates_rad_s = solution[10:13]
        - deltas = lookup from controller["deltas_history"] by timestamp

src/metrics.py::compute_tracking_metrics()
    - Identifies control window (start_idx to end_idx)
    - Computes per-axis and 3D tracking errors
    - Calculates fin saturation ratio
    - Extracts flight summary (apogee, max speed, etc.)

src/plots.py::generate_all_plots()
    - Full flight plots: 3D trajectory, 2D projections
    - Control phase plots: position tracking, errors, fin deflections,
      attitude (Euler), body rates

export_results():
    - Saves flight_history.csv
    - Saves flight_summary.csv
    - Saves metrics.json
    - Saves all plots to results/<run_id>/plots/
```

## Key Design Decisions

### 1. Local ENU Coordinates
All trajectory tracking uses local ENU with launch point as origin. This simplifies:
- Reference trajectory definition (start at 0,0,0)
- Error computation (direct vector subtraction)
- Comparison between reference and real trajectories

### 2. RocketPy Private Controller Infrastructure
Uses `rocketpy.control.controller._Controller` to integrate with RocketPy's ODE solver. This allows:
- Callback-based control at solver-determined timesteps
- Direct interaction with `GenericSurface` aerodynamic surfaces
- Access to full 13-state vector during control

**Caveat**: This is a private API. Future RocketPy versions may change or remove it.

### 3. Quaternion Attitude Representation
Quaternions (`[w, x, y, z]` format) are used for attitude to avoid gimbal lock. The controller works in the error quaternion space:
```
q_error = q_ref * conjugate(q_real)
```
PID is applied to the vector part `[x, y, z]` of `q_error`, which represents small-angle rotations around body axes.

### 4. FinAdapter Stateful Pattern
`FinAdapter` is a stateful wrapper that:
- Holds a reference to `controller_state["current_deltas"]`
- Converts 4 fin deflections to 6 aerodynamic coefficients (cL, cQ, cD, cm, cn, cl)
- Is called by RocketPy during aerodynamic force computation

This bridges the controller output (deltas) to the simulation (GenericSurface coefficients).

### 5. Control Window Detection
Plots and metrics focus on the "control phase" (when fins are actually deflecting). The control window is detected by:
```python
ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
start_idx = first nonzero deflection
end_idx = apogee (max altitude)
```

## Module Dependencies

```
main.py
├── config.py
├── initial_data.py
│   └── data/rockets/*.toml (via toml.load)
├── src/controllers.py
│   └── src/utils.py (quaternion math)
├── src/environment_builder.py
│   └── rocketpy.Environment
├── src/rocket_builder.py
│   ├── rocketpy.Rocket, rocketpy.SolidMotor
│   └── src/fin_model.py (FinAdapter)
│       └── rocketpy.Function (for GenericSurface coefficients)
├── src/simulation.py
│   ├── rocketpy.Flight
│   ├── src/plots.py
│   └── src/controllers.py (fin_controller)
├── src/metrics.py
│   ├── src/utils.py
│   └── src/reference.py
└── src/reference.py
    └── scipy.interpolate (for trajectory sampling)
```

## Data Structures

### Controller State Dict
```python
controller = {
    "integral_error": np.zeros(3),      # Accumulated PID integral
    "previous_error": np.zeros(3),      # Previous error for derivative
    "current_deltas": np.zeros(4),      # Latest fin deflections [d1,d2,d3,d4]
    "deltas_history": {}                # Dict: time -> [d1,d2,d3,d4]
}
```

### Flight History Entry
```python
{
    'time_s': float,
    'position_enu_m': np.array([x, y, z]),    # Local ENU (launch=0,0,0)
    'position_asl_m': np.array([x, y, z]),     # Absolute ASL
    'velocity_enu_m_s': np.array([vx, vy, vz]),
    'attitude_quaternion': np.array([w, x, y, z]),  # ENU→Body
    'body_rates_rad_s': np.array([p, q, r]),
    'deltas': np.array([d1, d2, d3, d4])
}
```

### Reference Trajectory
```python
reference = {
    'time_s': np.array([...]),
    'interpolators': {
        'x_enu_m': interp1d(...),
        'y_enu_m': interp1d(...),
        'z_enu_m': interp1d(...),
        'vx_enu_m_s': interp1d(...),
        'vy_enu_m_s': interp1d(...),
        'vz_enu_m_s': interp1d(...)
    },
    'peak_z_enu': float  # Maximum altitude in reference
}
```
