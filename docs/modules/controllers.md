# Module: `src/controllers.py`

## Overview

Implements the **fin deflection controller** for trajectory tracking using quaternion-based attitude control and PID control in the body frame. This is the core control logic that determines fin deflections based on reference trajectory error.

## Key Functions

### `fin_controller(t, state, controller, config, reference, gravity)`

**Purpose**: Main callback function for RocketPy's `_Controller` integration.

**Signature**:
```python
def fin_controller(
    t: float,
    state: np.ndarray,           # [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
    controller: dict,             # Controller state
    config: object,               # Config parameters
    reference: dict,              # Reference trajectory
    gravity: float               # Gravity magnitude (m/s²) from Environment
) -> np.ndarray:                  # [d1, d2, d3, d4] fin deflections (rad)
```

**Note**: The `gravity` parameter is obtained from `environment.gravity(elevation)` in `simulation.py` and passed to the callback.

**State Format** (RocketPy 13-state vector):
- `state[0:3]`: Position in ENU (m) - absolute ASL
- `state[3:6]`: Velocity in ENU (m/s)
- `state[6:10]`: Attitude quaternion `[w, x, y, z]` (ENU → Body)
- `state[10:13]`: Body angular rates `[p, q, r]` (rad/s)

**Control Logic Flow**:

1. **State Extraction**:
   ```python
    pos = np.array(state[0:3])      # ENU position (ASL)
    vel = np.array(state[3:6])       # ENU velocity
    q_real = np.array(state[6:10])   # Quaternion (ENU→Body)
    w_body = np.array(state[10:13])  # Body angular rates
    ```

2. **Control Activation Check**:
   ```python
    z_asl = pos[2]
    z_local = z_asl - config.elevation_asl_m
    vz = vel[2]
    
    # Check reference time limit
    ref_time_limit = reference['time_s'][-1]
    ref_sample = sample_reference(reference, t)
    vel_ref = ref_sample['velocity_enu_m_s']
    
    if (t < config.control_start_delay_s or 
        z_local < config.control_start_min_height_above_launch_m or 
        vz <= 0 or
        t > ref_time_limit or
        vel_ref[2] <= 0):
        deltas = np.zeros(4)
        controller["deltas_history"][float(t)] = deltas
        return deltas  # No control during burn, below rail, or descending
    ```

3. **Guidance (PD Control)**:
   ```python
    pos_ref = ref_sample['position_enu_m']  # Local ENU
    
    pos_local = pos - np.array([0, 0, config.elevation_asl_m])
    accel_cmd_enu = Kp_guidance * (pos_ref - pos_local) + Kd_guidance * (vel_ref - vel)
    accel_cmd_enu += np.array([0, 0, gravity])  # Gravity compensation (from Environment)
    ```

4. **Desired Attitude** (align nose with acceleration command):
   ```python
    q_ref = compute_desired_attitude(accel_cmd_enu, config)
    # Returns quaternion that rotates ENU [0,0,1] to align with accel_cmd
    ```

5. **Attitude PID** (in body frame):
   ```python
    q_error = quaternion_multiply(q_ref, quaternion_conjugate(q_real))
    error_vec = q_error[1:4]  # [e_roll, e_pitch, e_yaw]
    
    # PID on error vector
    controller["integral_error"] += error_vec * dt
    derivative = (error_vec - controller["previous_error"]) / dt
    controller["previous_error"] = error_vec
    
    u_pitch = Kp_attitude * error_vec[1] + Ki_attitude * controller["integral_error"][1] + Kd_attitude * derivative[1]
    u_yaw = Kp_attitude * error_vec[2] + Ki_attitude * controller["integral_error"][2] + Kd_attitude * derivative[2]
    ```

6. **Roll Control** (combined error and damping):
   ```python
    u_roll = config.Kp_roll * error_vec[0] - config.Kd_attitude * w_body[0]
    # Aligns roll to 0 (error_vec[0]) AND damps roll rate
    ```

7. **Mixer** (4 fins, cross configuration):
   ```python
    deltas = np.array([
        u_yaw + u_roll,     # Fin 1 (0°, right)
        u_pitch + u_roll,   # Fin 2 (90°, top)
        -u_yaw + u_roll,    # Fin 3 (180°, left)
        -u_pitch + u_roll   # Fin 4 (270°, bottom)
    ])
    ```

8. **Rate Limiting and Saturation**:
   ```python
    # Get limits from controller state (loaded from TOML)
    delta_max = controller["delta_max_rad"]
    delta_dot_max = controller["delta_dot_max_rad_s"]
    
    # Rate limit
    prev_deltas = controller.get("current_deltas", np.zeros(4))
    max_step = delta_dot_max * dt
    deltas = np.clip(deltas, prev_deltas - max_step, prev_deltas + max_step)
    
    # Saturation
    deltas = np.clip(deltas, -delta_max, delta_max)
    ```

9. **Update State**:
   ```python
    controller["current_deltas"] = deltas
    controller["deltas_history"][float(t)] = deltas
    ```

**Returns**: `deltas` (4-element array of fin deflections in radians)

---

### `build_controller(config)`

**Purpose**: Initializes the controller state dictionary.

**Signature**:
```python
def build_controller(config) -> dict:
```

**Returns**:
```python
{
    "integral_error": np.zeros(3),      # [roll, pitch, yaw] integral accumulator
    "previous_error": np.zeros(3),      # Previous error for derivative
    "current_deltas": np.zeros(4),      # Latest deltas [d1,d2,d3,d4]
    "deltas_history": {}                # Dict: time -> [d1,d2,d3,d4]
}
```

**Usage**: Called once in `main.py` to create the controller state object.

---

### `compute_desired_attitude(a_cmd_enu, config)`

**Purpose**: Computes the desired attitude quaternion that aligns the rocket's nose (Body Z-axis) with the commanded acceleration vector.

**Signature**:
```python
def compute_desired_attitude(a_cmd_enu: np.ndarray, config) -> np.ndarray:
```

**Mathematics**:
```
Input: a_cmd_enu (desired acceleration in ENU frame)
Output: q_ref (quaternion representing ENU → Body rotation)

If rocket nose is aligned with a_cmd_enu, then:
  Body Z-axis in ENU = a_cmd_enu (normalized)
  
We need q_ref such that:
  q_ref rotates ENU [0,0,1] to Body Z-axis in ENU
  
Equivalent: q_ref rotates ENU to Body, so Body Z = R(q_ref) * [0,0,1]
```

**Implementation**: Uses `utils.quaternion_from_vectors(direction, [0,0,1])` where `direction` is the normalized acceleration command.

**Returns**: Quaternion `[w, x, y, z]` (ENU → Body)

---

### `update_attitude_pid(state, q_ref, controller_state, config)`

**Purpose**: Standalone attitude PID function (alternative to inline logic in `fin_controller`).

**Signature**:
```python
def update_attitude_pid(
    state: dict,
    q_ref: np.ndarray,
    controller_state: dict,
    config: object
) -> dict:
```

**Input**:
- `state`: Dict with `attitude_quaternion` and `body_rates_rad_s`
- `q_ref`: Desired quaternion (ENU → Body)
- `controller_state`: Controller state dict
- `config`: Config object

**Output**:
```python
{
    "u_pitch": float,
    "u_yaw": float,
    "u_roll": float
}
```

**Note**: This function is defined but not currently used in the main control loop (the PID logic is inlined in `fin_controller` for clarity).

---

## Control Architecture

```
                    Reference Trajectory (ENU)
                             │
                             ▼
                    ┌────────────────┐
                    │  PD Guidance  │  ← Kp_guidance, Kd_guidance
                    └───────┬──────┘
                            │ accel_cmd_enu
                            ▼
                    ┌────────────────┐
                    │ Quaternion     │  ← compute_desired_attitude()
                    │ Attitude       │
                    └───────┬──────┘
                            │ q_ref (ENU→Body)
                            ▼
                    ┌────────────────┐
                    │ Attitude PID   │  ← Kp_att, Ki_att, Kd_att
                    │ (Body Frame)   │
                    └───────┬──────┘
                            │ u_pitch, u_yaw
                            │ (plus u_roll from damping)
                            ▼
                    ┌────────────────┐
                    │    Mixer       │  ← 4 fins, cross config
                    └───────┬──────┘
                            │ d1, d2, d3, d4
                            ▼
                    ┌────────────────┐
                    │   Saturation   │  ← clip to ±delta_max_rad
                    └───────┬──────┘
                            │
                            ▼
                    FinAdapter / GenericSurface
```

## Fin Configuration (Cross +)

Fins are numbered according to their angular position:

```
        Fin 2 (90°, Top)
           ↑ +Y
           │
           │
Fin 3 (180°) ←─┼─→ Fin 1 (0°, Right)
 (Left)       │
               ↓
        Fin 4 (270°, Bottom)
```

**Mixing Law**:
```
d1 = u_yaw + u_roll     (Fin 1: right)
d2 = u_pitch + u_roll   (Fin 2: top)
d3 = -u_yaw + u_roll    (Fin 3: left)
d4 = -u_pitch + u_roll  (Fin 4: bottom)
```

**Effect**:
- Positive `u_pitch` → Fin 2 up, Fin 4 down → Nose pitches up (body -Y direction)
- Positive `u_yaw` → Fin 1 up, Fin 3 down → Nose yaws right (body +X direction)
- Positive `u_roll` → All fins up → Roll moment (depends on fin aerodynamics)

## Quaternion Convention

**Attitude representation**: `[w, x, y, z]` (scalar-first)

**Frame**: ENU (East-North-Up) → Body (rocket frame)

**Error quaternion**:
```python
q_error = q_ref * conjugate(q_real)
# q_error represents rotation from current orientation to desired orientation
```

**Error vector mapping** (H4 convention):
- `q_error[1]` (x) → Roll error
- `q_error[2]` (y) → Pitch error
- `q_error[3]` (z) → Yaw error

**Small angle approximation**: For small errors, `q_error[1:4] ≈ [roll_error, pitch_error, yaw_error] / 2`

## Dependencies

- `numpy`: Numerical computations
- `src.utils`: Quaternion math (`quaternion_multiply`, `quaternion_conjugate`, `quaternion_from_vectors`)
- `src.reference`: `sample_reference()` for trajectory lookup

## Configuration Parameters Used

From `config.py`:
- `control_start_delay_s`: Control activation delay (default: 3.0s)
- `control_start_min_height_above_launch_m`: Min height for control (derived: `rail_length_m + safety_margin_m`)
- `Kp_guidance`, `Kd_guidance`: Guidance PD gains
- `Kp_attitude`, `Ki_attitude`, `Kd_attitude`: Attitude PID gains
- `Kp_roll`: Roll control gain (used with Kd_attitude for damping)
- `elevation_asl_m`: Launch elevation (for ENU conversion)
- `control_dt_s`: Timestep for derivative/integral calculations
- `rail_length_m`, `safety_margin_m`: Used to derive `control_start_min_height_above_launch_m`

From `controller_state` (loaded from TOML `[control_actuation]`):
- `delta_max_rad`: Maximum fin deflection (required/effective value from TOML, e.g., 0.349 rad ≈ 20°)
- `delta_dot_max_rad_s`: Maximum deflection rate (required/effective value from TOML, e.g., 5.236 rad/s)

## State Management

The `controller` dict is passed by reference and mutated during simulation:

```python
# Before simulation
controller = build_controller(config)

# During simulation (inside fin_controller callback)
controller["current_deltas"] = deltas          # Update latest
controller["deltas_history"][t] = deltas      # Store history
controller["integral_error"] += error * dt     # Accumulate integral
controller["previous_error"] = error          # Store for derivative
```

**Note**: `deltas_history` uses simulation time as key (float). Due to RocketPy's adaptive ODE solver, the actual callback times may not align perfectly with `control_dt_s`.

## Caveats

1. **Gravity compensation**: Simple `+ [0,0,9.81]` in guidance. May need tuning for non-vertical trajectories.

2. **Quaternion convention**: Ensure RocketPy uses the same `[w,x,y,z]` ENU→Body convention.

3. **Derivative calculation**: Uses finite difference with `control_dt_s`. Inaccurate if RocketPy calls the callback at different rates.

4. **Integral windup**: No anti-windup protection. Long durations with persistent error may cause large integral accumulation.

5. **Mixer assumptions**: Assumes cross (+) configuration with symmetric fin aerodynamics. Modify for different fin counts/angles.

6. **Roll control**: Simple P damping on roll rate. No roll position control (rocket can accumulate roll angle).
