# Coordinate Systems and Frame Conventions

## Overview

This document describes the coordinate systems used in the Rocket Control TFG project. All trajectory data (reference and real) uses **local ENU (East-North-Up)** coordinates with the launch pad as the origin.

## ENU Coordinate System

### Definition

**ENU (East-North-Up)** is a local tangent plane coordinate system:

- **Origin**: Launch pad position (latitude, longitude, elevation from `config.py`)
- **X-axis**: East (positive towards East)
- **Y-axis**: North (positive towards North)
- **Z-axis**: Up (positive away from Earth's center)

### Key Properties

| Property | Value |
|----------|-------|
| Origin | Launch pad: `(latitude, longitude, elevation)` |
| X | East (m) |
| Y | North (m) |
| Z | Up (m) |
| Initial position | `(0, 0, 0)` at launch |
| Initial velocity | `(0, 0, 0)` at launch |

### Why ENU?

1. **Intuitive**: "Up" is Z, matching the natural rocket ascent direction
2. **Local**: Trajectories are defined relative to launch site, not absolute geodetic coordinates
3. **Simplified tracking**: Reference trajectory starts at `(0,0,0)`, errors are direct vector differences
4. **RocketPy compatibility**: RocketPy's internal integration uses geodetic coordinates, but we convert to ENU for control

## Frame Transformations

### RocketPy Internal → Local ENU

RocketPy's `Flight` object stores state in geodetic/absolute coordinates:

```python
# RocketPy internal state (from flight.solution)
state = [x_geo, y_geo, z_asl, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
# Where: x_geo, y_geo are in ECEF or geodetic, z_asl is altitude ASL
```

We convert to local ENU by subtracting the launch position:

```python
# In src/simulation.py
launch_pos_enu = sol[0, 1:4]  # Initial position from RocketPy (absolute)

for i, t in enumerate(sol[:, 0]):
    state_vec = sol[i, 1:]  # [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
    
    # Convert to local ENU (launch pad = 0,0,0)
    pos_enu = state_vec[0:3] - launch_pos_enu
    
    history.append({
        'time_s': float(t),
        'position_enu_m': pos_enu,           # Local ENU (0,0,0) at launch
        'position_asl_m': state_vec[0:3],    # Absolute ASL for reference
        'velocity_enu_m_s': state_vec[3:6],  # Already in ENU-like frame
        ...
    })
```

**Important**: The launch position in ENU (`launch_pos_enu`) is extracted from the first timestep of RocketPy's solution. This ensures consistency between the simulation's internal coordinates and our local ENU frame.

## Attitude Representation

### Quaternion Convention

Attitude is represented as a **unit quaternion** in `[w, x, y, z]` format, representing rotation from **ENU (world) to Body (rocket)** frame.

```python
q = [w, x, y, z]  # ENU → Body rotation
# w: scalar part (cos(θ/2))
# x, y, z: vector part (sin(θ/2) * axis)
```

**RocketPy compatibility**: RocketPy's internal state uses the same `[w, x, y, z]` quaternion convention for ENU→Body rotation. This is verified in `src/simulation.py`:

```python
# RocketPy state: [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
# q0, q1, q2, q3 correspond to w, x, y, z in our convention
```

### Quaternion Operations

Located in `src/utils.py`:

#### Conjugate
```python
def quaternion_conjugate(q):
    """Returns conjugate of q = [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])
```

#### Multiplication
```python
def quaternion_multiply(q1, q2):
    """Multiplies q1 * q2 (apply rotation q2, then q1)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])
```

#### Error Quaternion
The attitude error for control is computed as:
```python
# q_error represents rotation from current orientation to desired orientation
q_error = q_ref * conjugate(q_real)
# where: q_ref is desired ENU→Body, q_real is actual ENU→Body
```

In the PID controller (`src/controllers.py`):
```python
q_error = utils.quaternion_multiply(q_ref, utils.quaternion_conjugate(q_real))
# Extract body-frame error components (H4 mapping)
error_vec = q_error[1:4]  # [e_roll, e_pitch, e_yaw] in body frame
```

#### Quaternion from Vectors
Used to compute desired attitude from acceleration command:
```python
def quaternion_from_vectors(v_from, v_to):
    """
    Returns quaternion representing rotation from v_from to v_to.
    Both vectors must be normalized.
    """
    # Implementation uses cross product and dot product
    # See src/utils.py for full implementation
```

### Euler Angles (for visualization)

Converted from quaternion for plotting using ZYX convention (roll, pitch, yaw):

```python
def quaternion_to_euler(q):
    """Converts quaternion [w, x, y, z] to Euler angles (roll, pitch, yaw)."""
    w, x, y, z = q
    
    # Roll (φ, rotation around X-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (θ, rotation around Y-axis)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) < 1 else np.sign(sinp) * np.pi/2
    
    # Yaw (ψ, rotation around Z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw
```

## Body Frame (Rocket)

The rocket's body frame is defined as:

- **X-axis**: Roll axis (points to the right, from center to Fin 1 at 0°)
- **Y-axis**: Pitch axis (points "up" relative to rocket, towards Fin 2 at 90°)
- **Z-axis**: Yaw axis / Longitudinal axis (points forward, along rocket nose)

### Fin Configuration (Cross +)

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

Mixing law in `src/controllers.py`:
```python
# u_pitch: virtual control for pitch (affects Fins 2 and 4)
# u_yaw: virtual control for yaw (affects Fins 1 and 3)
# u_roll: virtual control for roll (applied to all fins)

deltas = np.array([
    u_yaw + u_roll,     # Fin 1 (0°): right
    u_pitch + u_roll,   # Fin 2 (90°): top
    -u_yaw + u_roll,    # Fin 3 (180°): left
    -u_pitch + u_roll   # Fin 4 (270°): bottom
])
```

## Velocity and Angular Rates

### Velocity (ENU frame)
```python
velocity_enu_m_s = [vx, vy, vz]  # In local ENU coordinates
# vx: East velocity (m/s)
# vy: North velocity (m/s)
# vz: Up velocity (m/s)
```

### Body Angular Rates
```python
body_rates_rad_s = [p, q, r]  # In body frame (rad/s)
# p: roll rate (around X-axis)
# q: pitch rate (around Y-axis)
# r: yaw rate (around Z-axis)
```

## Reference Trajectory Format

The reference trajectory CSV uses ENU coordinates:

```csv
time_s,x_enu_m,y_enu_m,z_enu_m,vx_enu_m_s,vy_enu_m_s,vz_enu_m_s
0.0,0.0,0.0,0.0,0.0,0.0,100.0
0.01,0.0,0.0,1.0,0.0,0.0,100.0
...
```

- All positions are in **local ENU** relative to launch pad
- All velocities are in **local ENU** frame
- Time is in seconds from launch (`t=0` at launch)

Sampling the reference at arbitrary times:
```python
from src.reference import load_reference_trajectory, sample_reference

reference = load_reference_trajectory("data/trajectory/vertical.csv")
sample = sample_reference(reference, time_s=5.0)

# Returns:
# sample['position_enu_m'] = np.array([x, y, z])
# sample['velocity_enu_m_s'] = np.array([vx, vy, vz])
```

## Coordinate System Summary

| Entity | Frame | Coordinates | Origin |
|--------|-------|--------------|--------|
| Reference trajectory | ENU | `(x, y, z)` in meters | Launch pad |
| Real trajectory (output) | ENU | `(x_local_m, y_local_m, z_local_m)` | Launch pad |
| Real trajectory (internal) | Geodetic/ASL | `(x_geo, y_geo, z_asl)` | Earth center / sea level |
| Velocity | ENU | `(vx, vy, vz)` m/s | - |
| Attitude quaternion | ENU→Body | `[w, x, y, z]` | - |
| Body angular rates | Body | `(p, q, r)` rad/s | Rocket CG |
| Fin deflections | Body | `(d1, d2, d3, d4)` rad | Hinge line |

## Important Notes

1. **Launch position subtraction**: Always subtract `launch_pos_enu` from RocketPy's absolute positions to get local ENU.

2. **Quaternion order**: We use `[w, x, y, z]` (scalar-first), matching RocketPy's internal convention.

3. **Euler angle convention**: ZYX (roll around X, pitch around Y, yaw around Z).

4. **Fin sign convention**: Positive deflection follows the right-hand rule around the fin's hinge line (typically: positive = trailing edge moves "up" relative to rocket centerline).

5. **Control phase**: Control activates only when:
   - `t > control_start_delay_s` (default 3s, after motor burn)
   - `z_local > control_start_min_height_above_launch_m` (default 11m, above rail)
   - `vz > 0` (ascending phase only)
