# Module: `src/fin_model.py`

## Overview

Implements the **FinAdapter** class that bridges the controller's fin deflection commands to RocketPy's `GenericSurface` aerodynamic coefficients. This is the critical link between control logic and flight physics.

## Class: `FinAdapter`

### Constructor

```python
class FinAdapter:
    def __init__(self, controller_state: dict, actuation_params: dict):
```

**Parameters**:
- `controller_state`: Dict containing `"current_deltas"` (4-element array)
- `actuation_params`: Dict from TOML `[control_actuation]` section

**Stored Attributes**:
```python
self.controller_state = controller_state  # Reference to controller dict
self.params = actuation_params

# Aerodynamic derivatives (from actuation_params)
self.cN_delta = self.params.get("cN_delta_per_rad", 0.0)
self.cm_delta = self.params.get("cm_delta_per_rad", 0.0)
self.cy_delta = self.params.get("cy_delta_per_rad", 0.0)
self.cn_delta = self.params.get("cn_moment_delta_per_rad", 0.0)
self.cl_delta = self.params.get("cl_delta_per_rad", 0.0)
```

---

## Key Methods

### `get_current_deltas()`

**Purpose**: Retrieves the latest fin deflections from the controller state.

**Signature**:
```python
def get_current_deltas(self) -> np.ndarray:  # Shape: (4,)
```

**Implementation**:
```python
def get_current_deltas(self):
    return self.controller_state.get("current_deltas", np.zeros(4))
```

**Note**: The `controller_state["current_deltas"]` is updated by `src/controllers.py::fin_controller()` during the simulation callback.

---

### Coefficient Functions

These methods are called by RocketPy's `GenericSurface` during aerodynamic force computation. They must match the signature expected by `rocketpy.Function`:

```python
def coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate) -> float:
```

#### `cl_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Lift coefficient (cL) from fin deflection.

**Mixing Logic**:
```python
def cl_coeff(self, ...):
    deltas = self.get_current_deltas()
    delta_pitch = (deltas[1] - deltas[3]) / 2.0  # (Fin2 - Fin4) / 2
    
    # cL = -cN_delta * delta_pitch
    # Negative because GenericSurface uses -lift in aero frame
    return -self.cN_delta * delta_pitch
```

**Physics**: 
- Positive `delta_pitch` (Fin2 up, Fin4 down) → Nose pitches "up" (Body -Y)
- Lift (cL) in RocketPy aero frame points "up" relative to rocket
- Negative sign because `R2 = -lift` in GenericSurface's frame conversion

#### `cq_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Side force coefficient (cQ) from fin deflection.

**Mixing Logic**:
```python
def cq_coeff(self, ...):
    deltas = self.get_current_deltas()
    delta_yaw = (deltas[0] - deltas[2]) / 2.0  # (Fin1 - Fin3) / 2
    
    # cQ = cy_delta * delta_yaw
    return self.cy_delta * delta_yaw
```

**Physics**:
- Positive `delta_yaw` (Fin1 up, Fin3 down) → Nose yaws right (Body +X)
- Side force (cQ) in RocketPy aero frame points "right" relative to rocket

#### `cd_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Drag coefficient (cD) from fin deflection.

**Current Implementation**:
```python
def cd_coeff(self, ...):
    return 0.0  # No induced drag model yet
```

**Future Enhancement**: Add induced drag: `k * (cL² + cQ²)`

#### `cm_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Pitch moment coefficient (cm) from fin deflection.

```python
def cm_coeff(self, ...):
    deltas = self.get_current_deltas()
    delta_pitch = (deltas[1] - deltas[3]) / 2.0
    return self.cm_delta * delta_pitch
```

#### `cn_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Yaw moment coefficient (cn) from fin deflection.

```python
def cn_coeff(self, ...):
    deltas = self.get_current_deltas()
    delta_yaw = (deltas[0] - deltas[2]) / 2.0
    return self.cn_delta * delta_yaw
```

#### `cl_roll_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)`

**Purpose**: Roll moment coefficient (cl) from fin deflection.

```python
def cl_roll_coeff(self, ...):
    deltas = self.get_current_deltas()
    delta_roll = np.mean(deltas)  # (d1 + d2 + d3 + d4) / 4
    return self.cl_delta * delta_roll
```

**Physics**: Symmetric deflection (all fins up/down) produces roll moment.

---

### `get_coefficients_dict()`

**Purpose**: Returns a dictionary of `rocketpy.Function` objects for `GenericSurface`.

**Signature**:
```python
def get_coefficients_dict(self) -> dict:
```

**Implementation**:
```python
def get_coefficients_dict(self):
    inputs = ["alpha", "beta", "mach", "reynolds", "pitch_rate", "yaw_rate", "roll_rate"]
    return {
        "cL": Function(self.cl_coeff, inputs, ["cL"]),
        "cQ": Function(self.cq_coeff, inputs, ["cQ"]),
        "cD": Function(self.cd_coeff, inputs, ["cD"]),
        "cm": Function(self.cm_coeff, inputs, ["cm"]),
        "cn": Function(self.cn_coeff, inputs, ["cn"]),
        "cl": Function(self.cl_roll_coeff, inputs, ["cl"]),
    }
```

**Usage**:
```python
adapter = FinAdapter(controller_state, actuation_params)
coeffs = adapter.get_coefficients_dict()

control_surface = GenericSurface(
    reference_area=...,
    reference_length=...,
    coefficients=coeffs,
    name="Control Fins"
)
```

**How it works**:
1. `GenericSurface` calls `coeff_function(alpha, beta, mach, ...)` during aerodynamic computation
2. The function (`cl_coeff`, `cq_coeff`, etc.) reads `current_deltas` from `controller_state`
3. It computes the coefficient based on fin deflections and aerodynamic derivatives
4. Returns the coefficient value (float)

---

## Fin Mixing Laws

### Pitch Control (affects Fins 2 and 4)
```
delta_pitch = (deltas[1] - deltas[3]) / 2.0
cL = -cN_delta * delta_pitch
cm = cm_delta * delta_pitch
```

- Fin 2 (90°, top): `deltas[1]`
- Fin 4 (270°, bottom): `deltas[3]`
- Positive `delta_pitch` → Fin2 up, Fin4 down → Nose pitches "up" (Body -Y)

### Yaw Control (affects Fins 1 and 3)
```
delta_yaw = (deltas[0] - deltas[2]) / 2.0
cQ = cy_delta * delta_yaw
cn = cn_delta * delta_yaw
```

- Fin 1 (0°, right): `deltas[0]`
- Fin 3 (180°, left): `deltas[2]`
- Positive `delta_yaw` → Fin1 up, Fin3 down → Nose yaws right (Body +X)

### Roll Control (affects all fins)
```
delta_roll = mean(deltas)
cl = cl_delta * delta_roll
```

- Symmetric deflection (all fins up/down) → Pure roll moment

---

## RocketPy GenericSurface Aero Frame

Understanding the frame conversion is critical:

```
GenericSurface internal computation:
  R1, R2, R3 = rotation_matrix @ Vector([side, -lift, -drag])
  
Where:
  - side = cQ (side force coefficient)
  - lift = cL (lift coefficient)
  - drag = cD (drag coefficient)
  
Rotation matrix: Converts from aero frame to body frame
```

**Body Frame (RocketPy)**:
- X: Right (toward Fin 1)
- Y: "Up" (toward Fin 2)
- Z: Forward (along nose)

**Aero Frame (GenericSurface)**:
- X: Side (right)
- Y: -Lift (down, relative to rocket)
- Z: -Drag (backward)

**Sign Conventions**:
- `cL`: Positive = lift "up" (rocket +Y direction)
- `cQ`: Positive = side force right (rocket +X direction)
- In `GenericSurface`, `R2 = -lift`, so `cL` must be negative for positive lift

---

## Dependencies

- `numpy`: Array operations, mean calculation
- `rocketpy`: `Function` (for wrapping coefficient callbacks)

---

## Configuration Parameters

From `data/rockets/leon_2.toml`:

```toml
[control_actuation]
cN_delta_per_rad = 4.8           # Normal force derivative
cm_delta_per_rad = -25.2          # Pitch moment derivative
cy_delta_per_rad = 4.8            # Side force derivative
cn_moment_delta_per_rad = -25.2   # Yaw moment derivative
cl_delta_per_rad = 0.0            # Roll moment derivative
```

**Note**: `cm_delta` and `cn_delta` are typically negative for stability (fin deflection that pitches nose up produces nose-down moment).

---

## Caveats and Notes

1. **Stateful Design**: `FinAdapter` reads from `controller_state["current_deltas"]` which is updated asynchronously by the controller callback. Ensure thread safety if using parallel execution (not currently an issue with single-threaded RocketPy).

2. **No Rate Limiting**: The model doesn't include fin deflection rate limits (`delta_dot_max_rad_s` is defined in TOML but not used here). Rate limiting should be implemented in the controller.

3. **Simplified Aerodynamics**: No induced drag, no Mach/Reynolds effects on control derivatives. The coefficients are purely linear functions of fin deflection.

4. **Fixed Coefficients**: Assumes `cN_delta`, etc., are constant. Real fins have nonlinear aerodynamics at high alpha/beta or transonic speeds.

5. **No Hysteresis**: Fin deflection → force is assumed instantaneous. Real fins have dynamic effects (unsteady aerodynamics).

6. **Neutral Lift Drag**: `cd_coeff` returns 0.0. Add induced drag model: `cD = cD0 + k * (cL² + cQ²)`.

7. **GenericSurface Frame**: Ensure the sign conventions match your aerodynamic database. The current implementation assumes specific RocketPy frame conventions.

---

## Example Usage

```python
from src.fin_model import FinAdapter
from src.controllers import build_controller
import numpy as np

# Controller state (updated by controller)
controller_state = {
    "current_deltas": np.zeros(4),
    "integral_error": np.zeros(3),
    "previous_error": np.zeros(3),
    "deltas_history": {}
}

# Actuation parameters (from TOML)
actuation_params = {
    "cN_delta_per_rad": 4.8,
    "cm_delta_per_rad": -25.2,
    "cy_delta_per_rad": 4.8,
    "cn_moment_delta_per_rad": -25.2,
    "cl_delta_per_rad": 0.0
}

# Create adapter
adapter = FinAdapter(controller_state, actuation_params)

# Get coefficients dict for GenericSurface
coeffs = adapter.get_coefficients_dict()

# Simulate a pitch deflection
controller_state["current_deltas"] = np.array([0, 0.1, 0, -0.1])  # Pitch up

# Test coefficient calculation
alpha, beta, mach = 0.0, 0.0, 0.3
pitch_rate, yaw_rate, roll_rate = 0.0, 0.0, 0.0
reynolds = 1e6

cL = coeffs["cL"](alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)
print(f"cL = {cL}")  # Should be negative (see sign convention above)
```
