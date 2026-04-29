# Module: `src/rocket_builder.py`

## Overview

Constructs the **RocketPy Rocket and SolidMotor** objects from configuration data. Attaches aerodynamic surfaces including the controlled `GenericSurface` (rear fins) and passive trapezoidal fins for stability.

## Key Functions

### `build_rocket(case_data, config, controller_state)`

**Purpose**: Creates a fully configured RocketPy Rocket object ready for simulation.

**Signature**:
```python
def build_rocket(
    case_data: dict,
    config: object,
    controller_state: dict
) -> RocketPy.Rocket:
```

**Workflow**:

1. **Extract Parameters**:
   ```python
   params = case_data["rocket_params"]
   actuation = params["control_actuation"]
   geom = params["geometry"]
   ```

2. **Build Solid Motor**:
   ```python
   import pandas as pd
   motor_df = pd.read_csv(case_data["motor_path"], comment='#')
   thrust_data = motor_df.values
   
   motor = SolidMotor(
       thrust_source=thrust_data,
       dry_mass=2.0,  # Motor casing mass
       dry_inertia=(0.1, 0.1, 0.01),  # (Ixx, Iyy, Izz)
       nozzle_radius=0.033,
       throat_radius=0.011,
       grain_number=3,
       grain_density=1020,  # kg/m³
       grain_outer_radius=0.033,
       grain_initial_inner_radius=0.015,
       grain_initial_height=0.12,
       grain_separation=0.005,
       grains_center_of_mass_position=0.3,
       center_of_dry_mass_position=0.3,
       interpolation_method='linear',
       coordinate_system_orientation='nozzle_to_combustion_chamber'
   )
   ```

3. **Build Rocket Core**:
   ```python
   rocket = Rocket(
       radius=geom["radius_m"],
       mass=geom["dry_mass_kg"],
       inertia=(
           geom["inertia_yy_kg_m2"],  # Iyy
           geom["inertia_zz_kg_m2"],  # Izz
           geom["inertia_xx_kg_m2"]   # Ixx (roll)
       ),
       power_off_drag=case_data["drag_path"],
       power_on_drag=case_data["drag_path"],
       center_of_mass_without_motor=0,  # Set dynamically with motor
       coordinate_system_orientation='tail_to_nose'
   )
   
   rocket.add_motor(motor, position=-geom["length_m"])
   ```

4. **Add Nose Cone**:
   ```python
   rocket.add_nose(
       length=0.5,
       kind="vonKarman",
       position=0  # From nose tip (tail_to_nose coord)
   )
   ```

5. **Create Controlled Fins (GenericSurface)**:
   ```python
   from src.fin_model import FinAdapter
   
   adapter = FinAdapter(controller_state, actuation)
   coeffs = adapter.get_coefficients_dict()
   
   control_surface = GenericSurface(
       reference_area=actuation["reference_area_m2"],
       reference_length=actuation["reference_length_m"],
       coefficients=coeffs,
       name="Control Fins"
   )
   
   # Position relative to nose tip (tail_to_nose: 0 = nose, positive = toward tail)
   # leon_2.toml: fin_aerodynamic_center_x_m = 0.7 (from nose)
   rocket.add_surfaces(control_surface, -actuation["fin_aerodynamic_center_x_m"])
   ```

6. **Add Parachute**:
   ```python
   rocket.add_parachute(
       name='Main',
       cd_s=10.0,  # Drag area (m²)
       trigger='apogee'
   )
   ```

7. **Add Passive Stabilization Fins**:
   ```python
   f = params["fins"]
   rocket.add_trapezoidal_fins(
       n=f["count"],  # 4
       root_chord=f["root_chord_m"],
       tip_chord=f["tip_chord_m"],
       span=f["span_m"],
       position=-f["position_from_tail_m"],
       sweep_length=f["sweep_length_m"],
       cant_angle=f["cant_angle_deg"]
   )
   ```

**Returns**: Fully configured `RocketPy.Rocket` object.

---

## Coordinate Systems

### RocketPy Rocket Coordinates
- **Orientation**: `tail_to_nose` (0 = tail, positive = toward nose)
- **Positioning**: All `add_*` methods use this coordinate system

### GenericSurface Positioning
```python
# In leon_2.toml:
fin_aerodynamic_center_x_m = 0.7  # From nose tip

# Convert to tail_to_nose:
position_from_tail = -0.7  # Negative because tail_to_nose has 0 at nose

rocket.add_surfaces(control_surface, position_from_tail)
```

### Motor Positioning
```python
# Motor position: distance from rocket nose to motor tail
rocket.add_motor(motor, position=-geom["length_m"])  # Motor tail at rocket tail
```

---

## Control Actuation Parameters

From `data/rockets/leon_2.toml` → `actuation` dict:

| Parameter | Description | Units | Example |
|-----------|-------------|-------|---------|
| `reference_area_m2` | Fin reference area | m² | 0.01767 |
| `reference_length_m` | Fin reference length | m | 0.15 |
| `fin_aerodynamic_center_x_m` | AC from nose | m | 0.7 |
| `fin_aerodynamic_center_y_m` | AC radial arm | m | 0.15 |
| `cN_delta_per_rad` | Normal force derivative | 1/rad | 4.8 |
| `cm_delta_per_rad` | Pitch moment derivative | 1/rad | -25.2 |
| `cy_delta_per_rad` | Side force derivative | 1/rad | 4.8 |
| `cn_moment_delta_per_rad` | Yaw moment derivative | 1/rad | -25.2 |
| `cl_delta_per_rad` | Roll moment derivative | 1/rad | 0.0 |
| `delta_max_rad` | Max deflection | rad | 0.349 (20°) |

These are passed to `FinAdapter` which generates the coefficient functions for `GenericSurface`.

---

## Dependencies

- `pandas`: Reading motor thrust CSV
- `rocketpy`: `Rocket`, `SolidMotor`, `GenericSurface`
- `src.fin_model`: `FinAdapter` (stateful wrapper for GenericSurface coefficients)
- `toml`: (implicit, loaded by `initial_data.py`)

---

## Input Data Format

### Motor Thrust CSV
```csv
# Comments start with #
time_s,thrust_N
0.0,0.0
0.1,150.0
0.2,450.0
...
```

### Rocket TOML
See `docs/io_specs.md` for full format.

---

## Caveats and Notes

1. **Motor Dry Mass**: Hardcoded to 2.0 kg. Should be parameterized in TOML.

2. **Inertia Assumptions**: Motor dry inertia is hardcoded. Should be computed from motor geometry.

3. **Drag Coefficients**: Uses same drag curve for power-on and power-off. Real rockets have different drag with/without thrust.

4. **Parachute**: Simple apogee trigger. No backup triggers or staged recovery.

5. **Fin Placement**: Control fins positioned at `-fin_aerodynamic_center_x_m`. Ensure this matches the rocket's CG at motor burnout for stability.

6. **GenericSurface Integration**: The `FinAdapter` object (`adapter`) goes out of scope after `build_rocket()` returns, but the `coefficients` dict (created by `adapter.get_coefficients_dict()`) contains references to `adapter`'s methods. If Python garbage collection reclaims `adapter`, the callbacks may fail.
   
   **Fix**: Store `adapter` as an attribute of the rocket or in a longer-lived scope.

7. **Coordinate System Consistency**: Ensure `tail_to_nose` orientation is used consistently for all `add_*` calls.

---

## Example Usage

```python
import initial_data as init
import config as cfg
import src.rocket_builder as rocket_builder
import src.controllers as controllers

# Load data
case_data = init.load_initial_case_data()
config = cfg.load_config()

# Build controller state
controller_state = controllers.build_controller(config)

# Build rocket
rocket = rocket_builder.build_rocket(case_data, config, controller_state)

# Rocket is now ready for simulation
print(f"Rocket mass: {rocket.mass} kg")
print(f"Rocket radius: {rocket.radius} m")
```
