# Module: `src/simulation.py`

## Overview

Integrates with **RocketPy's 6-DOF flight simulation** using the private `_Controller` infrastructure. This module bridges the gap between RocketPy's ODE solver and the custom fin controller, executing the closed-loop simulation and extracting flight history.

## Key Functions

### `simulate_controlled_flight(rocket, environment, reference, controller, config)`

**Purpose**: Main simulation entry point. Sets up the RocketPy Flight object with closed-loop control and executes the simulation.

**Signature**:
```python
def simulate_controlled_flight(
    rocket: RocketPy.Rocket,
    environment: RocketPy.Environment,
    reference: dict,
    controller: dict,
    config: object
) -> list[dict]:  # Flight history
```

**Workflow**:

1. **Define Controller Callback**:
   ```python
   def controller_callback(t, sampling_rate, state, state_history, observed_vars, interactive_objs, sensors, env):
       # state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
       fin_controller(t, state, controller, config, reference)
       return None  # Must return None for RocketPy
   ```

2. **Find Control Surface**:
   ```python
   control_surf = None
   for item in rocket.aerodynamic_surfaces:
       if hasattr(item.component, 'name') and item.component.name == "Control Fins":
           control_surf = item.component
           break
   ```
   Locates the `GenericSurface` added by `rocket_builder.py` to pass to `_Controller`.

3. **Create RocketPy Controller**:
   ```python
   from rocketpy import Flight
   from rocketpy.control.controller import _Controller
   
   ctrl_obj = _Controller(
       interactive_objects=[control_surf] if control_surf else [],
       controller_function=controller_callback,
       sampling_rate=1.0/config.control_dt_s,  # e.g., 50 Hz
       name="Fin Controller"
   )
   
   rocket._add_controllers(ctrl_obj)  # Private API!
   ```

4. **Run Flight Simulation**:
   ```python
   flight = Flight(
       rocket=rocket,
       environment=environment,
       rail_length=config.rail_length_m,
       inclination=config.inclination_deg,
       heading=config.heading_deg,
       terminate_on_apogee=True,
       max_time=config.max_time_s,
       time_overshoot=False,  # High precision for control
       verbose=False
   )
   
   _ = flight.apogee_time  # Trigger simulation
   ```

5. **Extract Flight History**:
   ```python
   sol = np.array(flight.solution)  # Shape: (n_timesteps, 14) [t, state...]
   launch_pos_enu = sol[0, 1:4]  # Initial position (absolute)
   
   for i, t in enumerate(sol[:, 0]):
       state_vec = sol[i, 1:]  # [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz]
       
       # Convert to local ENU (launch = 0,0,0)
       pos_enu = state_vec[0:3] - launch_pos_enu
       
       # Lookup deltas from controller history
       ctrl_times = np.array(list(controller["deltas_history"].keys()))
       idx = (np.abs(ctrl_times - t)).argmin()
       deltas = controller["deltas_history"][ctrl_times[idx]]
       
       history.append({
           'time_s': float(t),
           'position_enu_m': pos_enu,
           'position_asl_m': state_vec[0:3],
           'velocity_enu_m_s': state_vec[3:6],
           'attitude_quaternion': state_vec[6:10],
           'body_rates_rad_s': state_vec[10:13],
           'deltas': deltas
       })
   ```

**Returns**: List of dicts, each representing a timestep (see "Flight History Format" below).

---

### `export_results(flight_history, reference, metrics, config)`

**Purpose**: Saves all simulation results to disk in a timestamped directory.

**Signature**:
```python
def export_results(
    flight_history: list[dict],
    reference: dict,
    metrics: dict,
    config: object
) -> str:  # run_dir path
```

**Workflow**:

1. **Create Output Directory**:
   ```python
   run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
   run_dir = os.path.join(config.results_dir, run_id)  # e.g., results/20260429_143052
   os.makedirs(run_dir, exist_ok=True)
   ```

2. **Save Metrics**:
   ```python
   with open(os.path.join(run_dir, "metrics.json"), "w") as f:
       json.dump(metrics, f, indent=4)
   ```

3. **Save Flight Summary**:
   ```python
   if "summary" in metrics:
       summary_df = pd.DataFrame([metrics["summary"]])
       summary_df.to_csv(os.path.join(run_dir, "flight_summary.csv"), index=False)
   ```

4. **Save Flight History** (flattened CSV):
   ```python
   flat_history = []
   for s in flight_history:
       item = {
           'time_s': s['time_s'],
           'x_local_m': s['position_enu_m'][0],
           'y_local_m': s['position_enu_m'][1],
           # ... (see io_specs.md for full format)
       }
       flat_history.append(item)
   
   df = pd.DataFrame(flat_history)
   df.to_csv(os.path.join(run_dir, "flight_history.csv"), index=False)
   ```

5. **Generate Plots**:
   ```python
   plots.generate_all_plots(flight_history, reference, metrics, config, run_dir)
   ```

**Output Structure**:
```
results/20260429_143052/
├── flight_history.csv          # Complete simulation state history
├── flight_summary.csv          # Key flight events and metrics
├── metrics.json                # Control performance metrics
├── controller_diagnostics.csv  # Per-sample controller audit trail
├── effective_config.json       # Config snapshot
├── manifest.json               # Asset hashes and git metadata
├── rocket_definition.toml      # Copy of TOML
├── rocket_artifacts.json       # Rocket stats
└── plots/
    ├── simulation/             # Full-flight trajectory plots
    │   ├── trajectory_3d.png
    │   ├── trajectory_2d_projections.png
    │   ├── rocket.png
    │   ├── static_margin.png
    │   └── motor_thrust.png
    └── control/                # Active-control-phase plots
        ├── position_per_axis.png
        ├── tracking_errors.png
        ├── fin_actuation.png
        ├── attitude_euler.png
        ├── body_rates.png
        ├── trajectory_3d.png
        └── trajectory_2d_projections.png
```

---

## Flight History Format

Each entry in `flight_history` is a dict:

```python
{
    'time_s': float,                    # Simulation time (s)
    'position_enu_m': np.array([x, y, z]),    # Local ENU (launch=0,0,0)
    'position_asl_m': np.array([x, y, z]),    # Absolute ASL
    'velocity_enu_m_s': np.array([vx, vy, vz]),  # ENU velocity
    'attitude_quaternion': np.array([w, x, y, z]),  # ENU→Body
    'body_rates_rad_s': np.array([p, q, r]),  # Body rates (rad/s)
    'deltas': np.array([d1, d2, d3, d4])     # Fin deflections (rad)
}
```

**Key Conventions**:
- `position_enu_m`: Local ENU with launch pad as origin (0,0,0)
- `position_asl_m`: Absolute altitude ASL (for reference/debugging)
- `attitude_quaternion`: `[w, x, y, z]` format (scalar-first)
- `deltas`: 4-element array for fins in cross configuration (0°, 90°, 180°, 270°)

---

## RocketPy Integration Details

### Private API Usage

**Warning**: This module uses RocketPy's private `_Controller` class:

```python
from rocketpy.control.controller import _Controller
rocket._add_controllers(ctrl_obj)
```

**Implications**:
- May break with future RocketPy versions
- Not officially supported by RocketPy team
- Provides direct access to the control loop during ODE integration

### Controller Callback Signature

RocketPy passes the following to the callback:

```python
def controller_callback(
    t: float,                    # Current simulation time (s)
    sampling_rate: float,         # Controller sampling rate (Hz)
    state: list,                  # 13-state vector [x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz]
    state_history: list,          # History of states (not used)
    observed_vars: list,          # Sensor observations (not used)
    interactive_objs: list,       # List of interactive objects (GenericSurface)
    sensors: list,                # List of sensors (not used)
    env: Environment              # RocketPy Environment object
) -> None:  # Must return None
```

**Important**: The callback modifies the `controller` dict in-place (via `fin_controller`), which updates `current_deltas`. The `FinAdapter` reads `current_deltas` when computing aerodynamic coefficients.

### State Vector Format

RocketPy's internal state (13 variables):

| Index | Variable | Description | Units | Frame |
|-------|----------|-------------|-------|-------|
| 0 | `x` | East position | m | ENU (absolute ASL) |
| 1 | `y` | North position | m | ENU (absolute ASL) |
| 2 | `z` | Up position | m | ENU (absolute ASL) |
| 3 | `vx` | East velocity | m/s | ENU |
| 4 | `vy` | North velocity | m/s | ENU |
| 5 | `vz` | Up velocity | m/s | ENU |
| 6 | `q0` | Quaternion w | - | ENU → Body |
| 7 | `q1` | Quaternion x | - | ENU → Body |
| 8 | `q2` | Quaternion y | - | ENU → Body |
| 9 | `q3` | Quaternion z | - | ENU → Body |
| 10 | `wx` | Roll rate | rad/s | Body X |
| 11 | `wy` | Pitch rate | rad/s | Body Y |
| 12 | `wz` | Yaw rate | rad/s | Body Z |

### Coordinate Conversion

RocketPy uses absolute geodetic coordinates internally. We convert to local ENU:

```python
launch_pos_enu = sol[0, 1:4]  # First timestep position (absolute)

# For each timestep:
pos_enu = state_vec[0:3] - launch_pos_enu  # Subtract launch position
```

This ensures all outputs use local ENU with origin at launch pad.

### Timestep Mismatch Issue

RocketPy's ODE solver may evaluate the controller at times that don't align with `control_dt_s`:

```python
# Controller history uses actual callback times:
controller["deltas_history"][float(t)] = deltas  # t is from RocketPy

# But flight.solution may have different timesteps (ODE solver adaptive):
sol[:, 0]  # These times may not match callback times exactly
```

**Solution**: We use **"latest command at or before solution time"** reconstruction with `np.searchsorted(side='right')`:

```python
idx = np.searchsorted(ctrl_times_sorted, t, side='right')
if idx > 0:
    deltas = ctrl_deltas_sorted[idx - 1]
else:
    deltas = np.zeros(4)  # No command yet
```

This ensures that:
- Rows before actual control activation keep zero deltas.
- No future command is assigned to an earlier solution timestamp.
- The exported history is truthful to the solver state at each node.

### Controller Diagnostics Export

Each run exports a `controller_diagnostics.csv` with per-sample audit data:

| Column | Description |
|--------|-------------|
| `time_s` | Callback timestamp |
| `control_active` | Whether control was active |
| `cutoff_reason` | Why control was inactive (if applicable) |
| `q_dynamic_pa` | Dynamic pressure |
| `airspeed_m_s` | Airspeed |
| `delta_limit_rad` | Effective authority limit (q-bar scheduled) |
| `effective_cD` | Control-induced drag coefficient |
| `raw_deltas_rad_*` | Raw deltas before rate/authority limiting |
| `limited_deltas_rad_*` | Final deltas after all limits |
| `position_error_enu_m_*` | Position error vector |
| `velocity_error_enu_m_s_*` | Velocity error vector |
| `attitude_error_quat_*` | Attitude error quaternion |
| `commanded_accel_enu_m_s2_*` | Commanded acceleration |

Duplicate callback detections are recorded with `cutoff_reason="duplicate_callback"`.

---

## Dependencies

- `numpy`: Array operations
- `pandas`: CSV export
- `json`: Metrics export
- `os`, `datetime`: Directory creation, timestamps
- `rocketpy`: `Flight`, `_Controller`
- `src.plots`: `generate_all_plots()`
- `src.controllers`: `fin_controller()`

---

## Configuration Parameters Used

From `config.py`:
- `control_dt_s`: Controller sampling period (determines `sampling_rate`)
- `rail_length_m`: Launch rail length
- `inclination_deg`: Launch inclination (90° = vertical)
- `heading_deg`: Launch heading (0° = North)
- `max_time_s`: Simulation timeout
- `results_dir`: Output directory base path

---

## Caveats and Known Issues

1. **Private API**: Uses `rocketpy.control.controller._Controller` and `rocketpy._add_controllers()`. These may change or be removed in future RocketPy versions.

2. **Duplicate Callbacks**: RocketPy may invoke the controller callback multiple times at the same timestamp. The controller is idempotent: duplicate calls return the existing command without advancing state. Duplicate detections are counted in `controller._duplicate_callback_count`.

3. **q-bar Authority Scheduling**: The controller uses a configurable q-bar scheduled deflection limit to prevent excessive drag at low dynamic pressure. Parameters are in `config.py` (`qbar_min_authority_pa`, `qbar_full_authority_pa`, `delta_max_qbar_min_rad`).

4. **Terminate-on-Apogee**: Set `config.terminate_on_apogee = True` for faster tuning runs (simulation ends at apogee). Default is `False` (full flight through descent).

5. **Quaternion Convention**: Assumes RocketPy uses `[w, x, y, z]` ENU→Body quaternions. Verify this matches your RocketPy version.

---

## Example Usage

```python
from src.simulation import simulate_controlled_flight, export_results
import config as cfg
import initial_data as init
import src.rocket_builder as rocket_builder
import src.environment_builder as env_builder
import src.reference as reference_mod
import src.controllers as controllers

# Load config and data
config = cfg.load_config()
case_data = init.load_initial_case_data(config)

# Build components
reference = reference_mod.load_reference_trajectory(config.reference_path)
controller = controllers.build_controller(config)
environment = env_builder.build_environment(case_data, config)
rocket = rocket_builder.build_rocket(case_data, config, controller)

# Run simulation
flight_history = simulate_controlled_flight(
    rocket=rocket,
    environment=environment,
    reference=reference,
    controller=controller,
    config=config
)

# Export results
metrics = ...  # Compute metrics separately
run_dir = export_results(flight_history, reference, metrics, config)
print(f"Results saved to {run_dir}")
```
