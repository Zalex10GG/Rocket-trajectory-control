# Module: `src/utils.py`

## Overview

Provides **quaternion math utilities** and helper functions for control window detection. This module is used extensively by the controller and plotting modules.

## Key Functions

### `quaternion_conjugate(q)`

**Purpose**: Returns the conjugate of a quaternion.

**Signature**:
```python
def quaternion_conjugate(q: np.ndarray) -> np.ndarray:  # [w, x, y, z] -> [w, -x, -y, -z]
```

**Mathematics**:
```
q = [w, x, y, z]
q* = [w, -x, -y, -z]
```

**Usage**: Used to compute the inverse rotation. If `q` rotates from frame A to frame B, `q*` rotates from B to A.

**Example**:
```python
q = np.array([0.707, 0.707, 0, 0])  # 90° rotation around X
q_conj = quaternion_conjugate(q)  # -90° rotation around X
```

---

### `quaternion_multiply(q1, q2)`

**Purpose**: Multiplies two quaternions (applies rotation q2, then q1).

**Signature**:
```python
def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
```

**Mathematics** (Hamilton product):
```
q1 = [w1, x1, y1, z1]
q2 = [w2, x2, y2, z2]

q1 * q2 = [
    w1*w2 - x1*x2 - y1*y2 - z1*z2,
    w1*x2 + x1*w2 + y1*z2 - z1*y2,
    w1*y2 - x1*z2 + y1*w2 + z1*x2,
    w1*z2 + x1*y2 - y1*x2 + z1*w2
]
```

**Order**: `q1 * q2` means apply `q2` first, then `q1`.

**Usage in controller**:
```python
# q_error = q_ref * conjugate(q_real)
# Rotates from current orientation to desired orientation
q_error = quaternion_multiply(q_ref, quaternion_conjugate(q_real))
```

---

### `quaternion_from_vectors(v_from, v_to)`

**Purpose**: Computes a quaternion representing rotation from `v_from` to `v_to`.

**Signature**:
```python
def quaternion_from_vectors(v_from: np.ndarray, v_to: np.ndarray) -> np.ndarray:
```

**Mathematics**:
1. Normalize input vectors
2. Compute dot product: `dot = v_from · v_to`
3. If `dot ≈ 1`: Return identity `[1, 0, 0, 0]`
4. If `dot ≈ -1`: Return 180° rotation around any orthogonal axis
5. Otherwise:
   - `axis = cross(v_from, v_to)`
   - `s = sqrt(2 * (1 + dot))`
   - `q = [s/2, axis[0]/s, axis[1]/s, axis[2]/s]`

**Usage**:
```python
# In src/controllers.py::compute_desired_attitude()
# Compute quaternion that aligns rocket nose (Body Z) with acceleration command
q_ref = quaternion_from_vectors(accel_cmd_enu, np.array([0, 0, 1]))
```

**Note**: This returns ENU → Body quaternion (consistent with RocketPy convention).

---

### `quaternion_to_euler(q)`

**Purpose**: Converts quaternion to Euler angles (roll, pitch, yaw) using ZYX convention.

**Signature**:
```python
def quaternion_to_euler(q: np.ndarray) -> tuple:  # (roll, pitch, yaw) in radians
```

**Mathematics** (ZYX convention):
```
q = [w, x, y, z]

Roll (φ, around X):
  sinr_cosp = 2 * (w*x + y*z)
  cosr_cosp = 1 - 2 * (x² + y²)
  roll = atan2(sinr_cosp, cosr_cosp)

Pitch (θ, around Y):
  sinp = 2 * (w*y - z*x)
  if |sinp| >= 1: pitch = sign(sinp) * π/2
  else: pitch = arcsin(sinp)

Yaw (ψ, around Z):
  siny_cosp = 2 * (w*z + x*y)
  cosy_cosp = 1 - 2 * (y² + z²)
  yaw = atan2(siny_cosp, cosy_cosp)
```

**Output**: `(roll, pitch, yaw)` in radians.

**Usage**:
```python
# In src/plots.py for attitude visualization
eulers = quaternion_to_euler(state['attitude_quaternion'])
# eulers[0] = roll (φ)
# eulers[1] = pitch (θ)
# eulers[2] = yaw (ψ)
```

---

### `get_control_window_indices(flight_history)`

**Purpose**: Identifies the start and end indices of the control phase in flight history.

**Signature**:
```python
def get_control_window_indices(flight_history: list[dict]) -> tuple:  # (start_idx, end_idx)
```

**Logic**:
1. Extract deltas and altitude from history
2. Identify control activation: `any(|delta| > 1e-6)`
3. Find start: First timestep with active control
4. Find end: Apogee (max altitude)

```python
deltas = np.array([s['deltas'] for s in flight_history])
pos_z = np.array([s['position_enu_m'][2] for s in flight_history])

ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
ctrl_active_indices = np.where(ctrl_active_mask)[0]

start_idx = ctrl_active_indices[0] if len(ctrl_active_indices) > 0 else 0
end_idx = np.argmax(pos_z)  # Apogee
```

**Returns**: `(start_idx, end_idx)` for slicing `flight_history`.

**Usage**:
```python
# In src/plots.py and src/metrics.py
start_idx, end_idx = get_control_window_indices(flight_history)
ctrl_history = flight_history[start_idx:end_idx+1]
```

**Note**: Control phase is from first nonzero fin deflection to apogee.

---

## Dependencies

- `numpy`: Array operations, math functions (`arctan2`, `arcsin`, `sqrt`, etc.)

---

## Quaternion Convention

All functions use **scalar-first** format: `[w, x, y, z]` where:
- `w`: Scalar (cos(θ/2))
- `x, y, z`: Vector (sin(θ/2) * axis)

**Frame**: ENU (East-North-Up) → Body (rocket frame)

**Normalization**: All quaternions are assumed to be unit quaternions (‖q‖ = 1).

---

## Caveats

1. **No normalization**: Functions assume input quaternions are already normalized. If not, results will be incorrect.

2. **Gimbal lock**: `quaternion_to_euler()` may have numerical issues near pitch = ±90°. This is a fundamental limitation of Euler angles.

3. **180° rotation**: `quaternion_from_vectors()` chooses an arbitrary orthogonal axis for 180° rotations. This is mathematically correct but may not be intuitive.

4. **Control window detection**: Uses a simple threshold (1e-6) to detect control activation. In theory, this could miss very small deflections or trigger on numerical noise.

5. **No input validation**: Functions don't check array shapes or quaternion validity. Passing wrong inputs will cause cryptic errors.

---

## Example Usage

```python
import numpy as np
from src.utils import quaternion_multiply, quaternion_conjugate, quaternion_to_euler

# Create a 90° yaw rotation (around Z)
q_yaw_90 = np.array([np.cos(np.pi/4), 0, 0, np.sin(np.pi/4)])  # [0.707, 0, 0, 0.707]

# Create a 90° pitch rotation (around Y)
q_pitch_90 = np.array([np.cos(np.pi/4), 0, np.sin(np.pi/4), 0])  # [0.707, 0, 0.707, 0]

# Combine: yaw 90°, then pitch 90°
q_combined = quaternion_multiply(q_pitch_90, q_yaw_90)

# Convert to Euler angles
roll, pitch, yaw = quaternion_to_euler(q_combined)
print(f"Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°, Yaw: {np.degrees(yaw):.1f}°")

# Inverse rotation
q_inv = quaternion_conjugate(q_yaw_90)  # -90° yaw
q_identity = quaternion_multiply(q_yaw_90, q_inv)  # Should be [1, 0, 0, 0]
```
