# Module: `src/reference.py`

## Overview

Handles **reference trajectory loading, interpolation, and sampling**. The reference trajectory defines the desired flight path that the controller attempts to track.

## Key Functions

### `load_reference_trajectory(path)`

**Purpose**: Loads reference trajectory from CSV and creates interpolators for continuous sampling.

**Signature**:
```python
def load_reference_trajectory(path: str) -> dict:
```

**Input CSV Format**:
```csv
time_s,x_enu_m,y_enu_m,z_enu_m,vx_enu_m_s,vy_enu_m_s,vz_enu_m_s
0.0,0.0,0.0,0.0,0.0,0.0,100.0
0.01,0.0,0.0,1.0,0.0,0.0,100.0
...
```

**Returns**:
```python
reference = {
    'time_s': np.array([...]),  # Time vector
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

**Implementation**:
```python
def load_reference_trajectory(path):
    df = pd.read_csv(path)
    times = df['time_s'].values
    
    reference = {
        'time_s': times,
        'interpolators': {}
    }
    
    for col in df.columns:
        if col != 'time_s':
            reference['interpolators'][col] = interp1d(
                times, df[col].values,
                kind='linear',
                fill_value='extrapolate'
            )
    
    # Peak finder
    if 'z_enu_m' in df.columns:
        reference['peak_z_enu'] = float(df['z_enu_m'].max())
    else:
        reference['peak_z_enu'] = 0.0
        
    return reference
```

**Important**: 
- Uses `scipy.interpolate.interp1d` with linear interpolation
- `fill_value='extrapolate'` allows sampling beyond the time range
- All coordinates are in **local ENU** (launch pad = 0,0,0)

---

### `sample_reference(reference, time_s)`

**Purpose**: Samples the reference trajectory at an arbitrary time.

**Signature**:
```python
def sample_reference(reference: dict, time_s: float) -> dict:
```

**Returns**:
```python
{
    'time_s': time_s,
    'x_enu_m': float,
    'y_enu_m': float,
    'z_enu_m': float,
    'vx_enu_m_s': float,
    'vy_enu_m_s': float,
    'vz_enu_m_s': float,
    'position_enu_m': np.array([x, y, z]),
    'velocity_enu_m_s': np.array([vx, vy, vz])
}
```

**Implementation**:
```python
def sample_reference(reference, time_s):
    sample = {'time_s': time_s}
    for col, interpolator in reference['interpolators'].items():
        sample[col] = float(interpolator(time_s))
    
    # Pack into vectors for convenience
    sample['position_enu_m'] = np.array([
        sample['x_enu_m'],
        sample['y_enu_m'],
        sample['z_enu_m']
    ])
    sample['velocity_enu_m_s'] = np.array([
        sample['vx_enu_m_s'],
        sample['vy_enu_m_s'],
        sample['vz_enu_m_s']
    ])
    
    return sample
```

**Usage**: Called by `src/controllers.py::fin_controller()` at each control timestep to get desired position and velocity.

---

### `compute_reference_acceleration(reference, time_s, dt=0.01)`

**Purpose**: Numerically computes reference acceleration from velocity (for advanced guidance laws).

**Signature**:
```python
def compute_reference_acceleration(reference: dict, time_s: float, dt: float = 0.01) -> np.ndarray:
```

**Mathematics**:
```
a(t) ≈ [v(t+dt) - v(t)] / dt
```

**Implementation**:
```python
def compute_reference_acceleration(reference, time_s, dt=0.01):
    v1 = sample_reference(reference, time_s)['velocity_enu_m_s']
    v2 = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
    return (v2 - v1) / dt
```

**Note**: This function is defined but not currently used in the main control loop (which uses simpler PD guidance).

---

## Coordinate Convention

**All reference trajectory values are in local ENU (East-North-Up)**:
- **Origin**: Launch pad position (0, 0, 0)
- **X**: East (m)
- **Y**: North (m)
- **Z**: Up (m)
- **Velocities**: In ENU frame (m/s)

**Why ENU?**:
1. Simplifies error computation: `error = pos_ref - pos_real`
2. Intuitive: Launch starts at origin
3. Reference trajectory generation is straightforward (e.g., vertical trajectory = constant x,y)

---

## Creating Reference Trajectories

### Vertical Trajectory (Default)

Use `src/gen_reference.py::generate_vertical_reference()`:

```python
from src.gen_reference import generate_vertical_reference

generate_vertical_reference(
    output_path="data/trajectory/vertical.csv",
    max_altitude=1000,  # Peak altitude (m)
    duration=20,         # Total trajectory duration (s)
    dt=0.01             # Time step (s)
)
```

**Creates**: Linear ascent to `max_altitude` in `duration/2`, then linear descent.

### Custom Trajectory

Create a CSV with columns: `time_s, x_enu_m, y_enu_m, z_enu_m, vx_enu_m_s, vy_enu_m_s, vz_enu_m_s`

**Example** (circular trajectory):
```python
import numpy as np
import pandas as pd

t = np.arange(0, 10, 0.01)
radius = 100  # m
omega = 0.1   # rad/s

df = pd.DataFrame({
    'time_s': t,
    'x_enu_m': radius * np.cos(omega * t),
    'y_enu_m': radius * np.sin(omega * t),
    'z_enu_m': 500 + 10 * t,  # Slowly ascending
    'vx_enu_m_s': -radius * omega * np.sin(omega * t),
    'vy_enu_m_s': radius * omega * np.cos(omega * t),
    'vz_enu_m_s': np.ones_like(t) * 10
})

df.to_csv("data/trajectory/circular.csv", index=False)
```

---

## Dependencies

- `numpy`: Array operations
- `pandas`: CSV reading
- `scipy.interpolate`: `interp1d` for continuous sampling

---

## Configuration

Reference path is set in `config.py`:
```python
config.reference_path = "data/trajectory/vertical.csv"
```

---

## Caveats and Notes

1. **Linear Interpolation**: Uses `kind='linear'` for simplicity. For smoother trajectories, consider `kind='quadratic'` or `kind='cubic'`.

2. **Extrapolation**: `fill_value='extrapolate'` allows sampling beyond the time range. This may produce unrealistic values if sampling far outside the trajectory.

3. **Velocity Continuity**: Linear interpolation of position → velocity is piecewise constant (not smooth). For smooth velocity, use higher-order interpolation or include velocity columns directly.

4. **Sampling Rate**: The reference is sampled at each controller callback (potentially ~50 Hz). Ensure the reference CSV has sufficient temporal resolution (e.g., `dt <= 0.01s`).

5. **Peak Altitude**: `reference['peak_z_enu']` is computed but not currently used. Could be used for apogee prediction or control termination.

6. **No Acceleration Column**: The CSV doesn't include acceleration. If needed for feedforward control, compute numerically via `compute_reference_acceleration()`.

7. **Single Trajectory**: Currently loads one reference trajectory. For time-varying or adaptive references, would need to modify the loading mechanism.

---

## Example Usage

```python
from src.reference import load_reference_trajectory, sample_reference
import numpy as np

# Load reference
reference = load_reference_trajectory("data/trajectory/vertical.csv")

# Sample at specific time
sample = sample_reference(reference, time_s=5.0)
print(f"Position at t={sample['time_s']}s: {sample['position_enu_m']}")
print(f"Velocity: {sample['velocity_enu_m_s']}")

# Sample at multiple times
times = np.linspace(0, 10, 100)
positions = np.array([sample_reference(reference, t)['position_enu_m'] for t in times])

# Get peak altitude
peak = reference['peak_z_enu']
print(f"Reference peak altitude: {peak} m")
```
