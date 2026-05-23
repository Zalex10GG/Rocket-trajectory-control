# Module: `src/rocket_builder.py`

## Overview

`src.rocket_builder` constructs the RocketPy rocket, motor, passive fins, active control surface, optional parachute, and output artifacts.

## `build_rocket(case_data, config, controller_state)`

The builder reads the rocket TOML from `case_data["rocket_params"]`.

It creates:

- `rocketpy.GenericMotor` from the motor TOML parameters and thrust CSV.
- `rocketpy.Rocket` from body mass, inertia, radius, drag data, and center of mass.
- Nose cone from the TOML `nosecone` section.
- Active `rocketpy.GenericSurface` controlled by `src.fin_model.FinAdapter`.
- Optional parachute from the TOML `parachute` section.
- Passive trapezoidal fins from the TOML `fins` section.

## Active Control Surface

`FinAdapter` builds the GenericSurface coefficient dictionary. The active surface is added at:

```text
fins.position_from_tail_m - fins.center_of_pressure_m
```

The GenericSurface uses:

- `reference_area_m2`
- `reference_length_m`
- control coefficient functions from `FinAdapter`
- `CONTROL_SURFACE_NAME`

## Controller State Population

The builder copies TOML actuation values into `controller_state`:

- `delta_max_rad`
- `delta_dot_max_rad_s`
- `cN_delta`
- `cy_delta`
- `k_drag_induced`

It also derives:

```python
config.control_start_min_height_above_launch_m = config.rail_length_m + config.safety_margin_m
```

## Output Artifacts

`export_rocket_creation_artifacts(...)` writes:

- `effective_config.json`
- `rocket_definition.toml`
- `rocket_artifacts.json`

## RocketPy Plot Patch

`_FixedRocketPlots` patches RocketPy's rocket drawing so fin root chords remain visible in the saved rocket diagram.
