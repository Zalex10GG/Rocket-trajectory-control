# Module: `src/fin_model.py`

## Overview

`src.fin_model` connects controller fin commands to RocketPy `GenericSurface` aerodynamic coefficients.

RocketPy internally copies objects during flight setup. To keep every copied coefficient function connected to the live controller state, the module stores a module-level controller-state reference with `set_controller_state_ref(controller_state)`.

## Controller State Reference

- `set_controller_state_ref(controller_state)`: registers the live controller dictionary.
- `get_controller_state()`: returns the registered dictionary.

`src.rocket_builder.build_rocket()` calls `set_controller_state_ref(...)` before simulation.

## `FinAdapter`

`FinAdapter` receives:

- the mutable controller state
- TOML `[control_actuation]` parameters

It reads the latest fin commands from:

```python
controller["current_deltas"]
```

## Coefficients

The adapter exposes RocketPy `Function` objects for:

- `cL`: lift force coefficient increment from pitch deflection
- `cQ`: side force coefficient increment from yaw deflection
- `cD`: induced drag from control forces
- `cm`: pitch moment coefficient, returned as zero
- `cn`: yaw moment coefficient, returned as zero
- `cl`: roll moment coefficient from mean fin deflection

Pitch, yaw, and roll deflection components are extracted with the same
Siouris cruciform-fin convention used by `src.controllers`. The array is
zero-indexed in code, so `delta0..delta3` correspond to physical fins
`d1..d4`:

```text
delta_pitch = (delta0 - delta2) / 2
delta_yaw   = (delta1 - delta3) / 2
delta_roll  = mean(delta0, delta1, delta2, delta3)
```

Pitch and yaw moments are produced by RocketPy from the applied control forces and the control surface position relative to the center of mass.
