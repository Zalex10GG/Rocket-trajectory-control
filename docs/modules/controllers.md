# Module: `src/controllers.py`

## Overview

`src.controllers` contains the closed-loop fin controller registered through RocketPy's private `_Controller` hook. The controller is implemented as a stateful callback around a mutable controller dictionary created by `build_controller(config)`.

## Controller State

`build_controller(config)` returns a dictionary with:

- pitch, yaw, and roll integral error state
- current fin deflections
- deflection history
- desired attitude quaternion history
- dynamic-pressure history
- controller diagnostics
- actuation limits copied later from the rocket TOML

`src.rocket_builder.build_rocket()` fills TOML-derived values such as `delta_max_rad`, `delta_dot_max_rad_s`, `cN_delta`, `cy_delta`, and `k_drag_induced`.

## Activation Conditions

`fin_controller(...)` commands zero fin deflection until all activation conditions are satisfied:

- current time is after `config.control_start_delay_s`
- rocket height above launch is at least `config.control_start_min_height_above_launch_m`
- simulated rocket vertical velocity is positive
- current time is within the reference duration
- reference vertical velocity is positive
- dynamic pressure is above `config.q_min_cutoff_pa`

When inactive, the controller records diagnostics and keeps `current_deltas` at zero.

## Guidance

The controller samples the reference at the current time and computes:

```text
position error = reference position - current position
velocity error = reference velocity - current velocity
```

The guidance command uses:

- `Kp_direction_guidance`
- `Kd_direction_guidance`
- wind compensation through `K_wind_comp` when wind is enabled
- correction limiting through `max_attitude_correction_deg`
- commanded angle-of-attack limiting through `max_commanded_aoa_deg`

The resulting commanded nose direction is converted to an ENU-to-body attitude quaternion with `compute_desired_attitude()`.

## Attitude PID

The attitude loop uses the error quaternion:

```text
q_error = q_ref * conjugate(q_real)
```

The controller maps quaternion vector components to:

- pitch error: `q_error[1]`
- yaw error: `q_error[2]`
- roll error: `q_error[3]`

Pitch and yaw use `Kp_attitude`, `Ki_attitude`, and `Kd_attitude`. These are calculated properties from `config.py`:

```python
Kp_attitude = Kp_attitude_zn * attitude_gain_scale
Ki_attitude = Ki_attitude_zn * attitude_gain_scale
Kd_attitude = Kd_attitude_zn * attitude_gain_scale
```

Roll uses `Kp_roll`, `Ki_roll`, and `Kd_roll`.

## Dynamic-Pressure Gain Scheduling

When `enable_gain_scheduling` is true, the attitude and roll gains are scaled by:

```text
q_scale = qbar_ref_pa / max(q_dynamic, q_min_cutoff_pa)
```

`q_scale` is capped by `gain_scheduling_max_scale`.

## Mixer

The mixer follows the Siouris cruciform-fin convention and maps pitch, yaw,
and roll control outputs to four fin deflections. The array is zero-indexed in
code, so `delta0..delta3` correspond to physical fins `d1..d4`:

```text
delta0 =  pitch + roll
delta1 =  yaw   + roll
delta2 = -pitch + roll
delta3 = -yaw   + roll
```

The same convention is used by `src.fin_model.FinAdapter` when extracting
pitch, yaw, and roll deflection components from `current_deltas`:

```text
pitch = (delta0 - delta2) / 2
yaw   = (delta1 - delta3) / 2
roll  = mean(delta0, delta1, delta2, delta3)
```

## Limits And Anti-Windup

The controller applies:

- conditional integration anti-windup
- optional first-order command smoothing from `actuator_command_filter_tau_s`
- rate limiting using `delta_dot_max_rad_s`
- dynamic-pressure authority limiting through `_compute_qbar_authority_limit(...)`

The q-bar authority schedule uses the configured low, full, and high dynamic-pressure thresholds and returns the live fin deflection limit used for saturation and diagnostics.

## Key Functions

- `fin_controller(...)`: RocketPy callback implementing guidance, attitude control, mixing, limits, and diagnostics.
- `build_controller(config)`: initializes the controller state dictionary.
- `compute_desired_attitude(a_cmd_enu)`: returns an ENU-to-body quaternion that aligns body +Z with the command direction.
- `compute_nose_direction_command(...)`: computes and limits the desired nose direction.
- `_compute_qbar_authority_limit(...)`: computes the live deflection authority limit.
