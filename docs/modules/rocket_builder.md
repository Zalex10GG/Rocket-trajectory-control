# Module: `src/rocket_builder.py`

## Overview

Constructs the **RocketPy Rocket and GenericMotor** objects from configuration data. Attaches aerodynamic surfaces including the controlled `GenericSurface` (rear fins) and passive trapezoidal fins for stability.

## Key Functions

### `build_rocket(case_data, config, controller_state)`

**Purpose**: Creates a fully configured RocketPy Rocket object ready for simulation.

**Signature**:
```python
def build_rocket(
    case_data: dict,
    config: object,
    controller_state: dict,
    return_components: bool = False
) -> RocketPy.Rocket:
```

**Workflow**:

1. **Extract Parameters**:
   ```python
    params = case_data["rocket_params"]
    actuation = params["control_actuation"]
    geom = params["body"]  # Note: [geometry] section renamed to [body] in TOML
    motor_params = params["motor"]  # Motor params from [motor] section
    ```

2. **Build Generic Motor** (replaces deprecated SolidMotor):
   ```python
    import pandas as pd
    motor_df = pd.read_csv(case_data["motor_path"], comment='#')
    thrust_data = motor_df.values
    
    motor = GenericMotor(
        thrust_source=thrust_data,
        burn_time=(motor_params["burn_time_start_s"], motor_params["burn_time_end_s"]),
        chamber_radius=motor_params["chamber_radius_m"],
        chamber_height=motor_params["chamber_height_m"],
        chamber_position=motor_params["chamber_position_m"],
        propellant_initial_mass=motor_params["propellant_initial_mass_kg"],
        nozzle_radius=motor_params["nozzle_radius_m"],
        dry_mass=motor_params["dry_mass_kg"],
        center_of_dry_mass_position=motor_params["center_of_dry_mass_position_m"],
        dry_inertia=(
            motor_params["dry_inertia_yy_kg_m2"],
            motor_params["dry_inertia_zz_kg_m2"],
            motor_params["dry_inertia_xx_kg_m2"]
        ),
        nozzle_position=motor_params["nozzle_position_m"],
        interpolation_method='linear',
        coordinate_system_orientation=motor_params["coordinate_system_orientation"]
    )
    ```

3. **Build Rocket Core**:
   ```python
    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        inertia=(geom["inertia_yy_kg_m2"],  # Iyy
                geom["inertia_zz_kg_m2"],  # Izz (note: now 0.01 for Leon 2)
                geom["inertia_xx_kg_m2"]   # Ixx (roll)
        ),
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=geom["center_of_mass_without_motor_m"],
        coordinate_system_orientation=geom["coordinate_system_orientation"]
    )
    
    rocket.add_motor(motor, position=-geom["length_m"])
    ```

4. **Add Nose Cone** (from `[nosecone]` section in TOML):
   ```python
    rocket.add_nose(
        length=params["nosecone"]["length_m"],
        kind=params["nosecone"]["kind"],
        position=params["nosecone"]["position_m"]  # From nose tip (tail_to_nose coord)
    )
    ```

5. **Create Controlled Fins (GenericSurface)**:
   ```python
    from src.fin_model import FinAdapter
    from src.constants import CONTROL_SURFACE_NAME
    
    adapter = FinAdapter(controller_state, actuation)
    coeffs = adapter.get_coefficients_dict()
    
    # Store actuation limits in controller state (from TOML, not config)
    controller_state["delta_max_rad"] = actuation["delta_max_rad"]
    controller_state["delta_dot_max_rad_s"] = actuation["delta_dot_max_rad_s"]
    
    # Derive minimum control activation height
    config.control_start_min_height_above_launch_m = config.rail_length_m + config.safety_margin_m
    
    control_surface = GenericSurface(
        reference_area=actuation["reference_area_m2"],
        reference_length=actuation["reference_length_m"],
        coefficients=coeffs,
        name=CONTROL_SURFACE_NAME  # "Control Fins"
    )
    
    # Position using fins section (position_from_tail_m from tail, negative for nose-ref)
    rocket.add_surfaces(control_surface, -params["fins"]["position_from_tail_m"])
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
        sweep_length=None,  # Using sweep_angle instead
        sweep_angle=f["sweep_angle_deg"],
        cant_angle=f["cant_angle_deg"]
    )
    ```

**Returns**: Fully configured `RocketPy.Rocket` object.

### `export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data)`

**Purpose**: Exports metadata about the rocket and its environment to the run directory.

**Outputs**:
- `effective_config.json`: Serializable config parameters
- `manifest.json`: Timestamp, git metadata, file hashes
- `rocket_definition.toml`: Copy of the rocket TOML
- `rocket_artifacts.json`: Basic rocket stats (mass, CoM, radius, components)

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

From `data/rockets/leon_2.toml` → `actuation` dict (from `[control_actuation]` section):

| Parameter | Description | Units | Example |
|-----------|-------------|-------|---------|
| `reference_area_m2` | Fin reference area | m² | 0.00785 |
| `reference_length_m` | Fin reference length (MAC) | m | 0.1452 |
| `cN_delta_per_rad` | Normal force derivative | 1/rad | 9.34 |
| `cy_delta_per_rad` | Side force derivative | 1/rad | 9.34 |
| `cl_delta_per_rad` | Roll moment derivative | 1/rad | 0.5 |
| `k_drag_induced` | Induced drag factor | - | 0.296 |
| `delta_max_rad` | Max deflection | rad | 0.349 (20°) |
| `delta_dot_max_rad_s` | Max deflection rate | rad/s | 5.236 |

**Note**: `cm_delta` and `cn_delta` are NOT used because:
1. `cm_coeff()` and `cn_coeff()` return 0.0 in `fin_model.py`
2. RocketPy computes moments automatically using the CP-to-CG moment arm
3. This avoids double-counting moments

**Fin position**: Control fins are positioned using `params["fins"]["position_from_tail_m"]` (not `fin_aerodynamic_center_x_m` which is obsolete).

These are passed to `FinAdapter` which generates the coefficient functions for `GenericSurface`.

---

## Dependencies

- `pandas`: Reading motor thrust CSV
- `rocketpy`: `Rocket`, `GenericMotor`, `GenericSurface`
- `src.fin_model`: `FinAdapter` (stateful wrapper for GenericSurface coefficients)
- `src.constants`: `CONTROL_SURFACE_NAME`
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

1. **Motor Parameters**: Now loaded from `[motor]` section in TOML (not hardcoded).

2. **Inertia Values**: Leon 2 has `inertia_zz_kg_m2 = 0.01` (very low for yaw). Check if this is physically realistic.

3. **Drag Coefficients**: Uses same drag curve for power-on and power-off. Real rockets have different drag with/without thrust.

4. **Parachute**: Simple apogee trigger. No backup triggers or staged recovery.

5. **Fin Placement**: Control fins positioned at `-params["fins"]["position_from_tail_m"]`. Ensure this matches the rocket's CG at motor burnout for stability.

6. **GenericSurface Integration**: The `FinAdapter` object (`adapter`) goes out of scope after `build_rocket()` returns, but the `coefficients` dict (created by `adapter.get_coefficients_dict()`) contains references to `adapter`'s methods. If Python garbage collection reclaims `adapter`, the callbacks may fail.
    
   **Fix**: Store `adapter` as an attribute of the rocket or in a longer-lived scope.

7. **Coordinate System Consistency**: Ensure `tail_to_nose` orientation is used consistently for all `add_*` calls.

8. **Nose Cone**: Now loaded from `[nosecone]` section (not hardcoded values).

---

## Example Usage

```python
import initial_data as init
import config as cfg
import src.rocket_builder as rocket_builder
import src.controllers as controllers

# Load data
config = cfg.load_config()
case_data = init.load_initial_case_data(config)

# Build controller state
controller_state = controllers.build_controller(config)

# Build rocket
rocket = rocket_builder.build_rocket(case_data, config, controller_state)

# Rocket is now ready for simulation
print(f"Rocket mass: {rocket.mass} kg")
print(f"Rocket radius: {rocket.radius} m")
```
