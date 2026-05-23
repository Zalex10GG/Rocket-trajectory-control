# Module: `src/utils.py`

## Overview

`src.utils` contains quaternion helpers, Euler conversions, and control-window detection helpers used by metrics and plots.

## Quaternion Helpers

The project uses scalar-first quaternions:

```text
[w, x, y, z]
```

Key helpers:

- `quaternion_conjugate(q)`
- `quaternion_multiply(q1, q2)`
- `quaternion_from_vectors(v_from, v_to)`
- `quaternion_to_matrix(q)`
- `quaternion_to_euler(q)`
- `rocketpy_quaternion_to_aerospace_euler(q, maps_body_to_enu=True)`
- `euler_to_quaternion(roll, pitch, yaw)`
- `compute_body_rates_from_quaternions(...)`

## Control Windows

- `get_control_window_indices(flight_history, controller_state=None)`: returns the ascent window from control start to apogee.
- `get_active_control_window_indices(flight_history, controller_state=None)`: returns the active-control window only.

When controller diagnostics are available, active-control detection uses `control_active` entries. Otherwise, it falls back to nonzero fin deflections.
