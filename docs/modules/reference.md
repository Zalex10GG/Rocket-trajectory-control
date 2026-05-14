# Module: `src/reference.py`

## Overview

Handles the loading, interpolation, and sampling of the target trajectory that the controller attempts to track.

## Data Format

The reference trajectory is stored in a CSV file with the following structure:
- `time_s`: Simulation time.
- `x_enu_m, y_enu_m, z_enu_m`: Position in the ENU frame.
- `vx_enu_m_s, vy_enu_m_s, vz_enu_m_s`: Velocity in the ENU frame.

## Key Functions

### `load_reference_trajectory(path)`
Loads the CSV and creates `interp1d` objects for each state variable. This allows the controller to sample the reference at any arbitrary timestamp during the integration process.

### `sample_reference(reference, time_s)`
Returns the interpolated position and velocity vectors for a given time $t$.

### `compute_reference_acceleration(reference, time_s)`
Computes the reference acceleration $\vec{a}_{ref}$ through numerical differentiation of the velocity:

$$\vec{a}_{ref}(t) \approx \frac{\vec{v}_{ref}(t + \Delta t) - \vec{v}_{ref}(t)}{\Delta t}$$

## Coordinate System

The reference trajectory is strictly defined in the **local ENU** frame:
- **Origin $(0,0,0)$**: Launch pad.
- **X**: East.
- **Y**: North.
- **Z**: Up.

This ensures that the error computation in the controller is direct:
$$\vec{e}_{pos} = \vec{p}_{ref} - \vec{p}_{real}$$
$$\vec{e}_{vel} = \vec{v}_{ref} - \vec{v}_{real}$$
