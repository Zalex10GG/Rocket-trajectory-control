# Module: `src/rocket_builder.py`

## Overview

Assembles the virtual rocket in RocketPy by integrating mass properties, motor characteristics, and aerodynamic surfaces.

## Rocket Assembly

The builder pulls all physical data from the TOML file:
1. **Core**: Initializes the `Rocket` with mass and inertia tensors.
2. **Motor**: Adds a `SolidMotor` using the thrust curve CSV and grain properties.
3. **Aero Surfaces**: Adds the nose cone and fins.
4. **Control**: Integrates the `GenericSurface` using a `FinAdapter` to allow real-time control.

## Longitudinal Axis Convention

RocketPy uses a coordinate system along the longitudinal axis. This project follows the **Tail-to-Nose** convention:
- **Tail**: Positioned at $0$ (or negative values in some contexts).
- **Nose**: Positioned at the total length of the rocket.

## Component Integration

### `build_rocket`
The main entry point for rocket construction. It:
- Sets the center of mass and inertia tensors.
- Loads the motor from `config.motor_path`.
- Configures the `GenericSurface` for active control using coefficients from the `[control_actuation]` TOML section.

### `FinAdapter`
Acts as the bridge between the controller and the physics engine. It defines the aerodynamic coefficients as functions of the current fin deflections $\delta$:

$$C_{N} = C_{N_{\delta}} \cdot \delta_{pitch}$$
$$C_{y} = C_{y_{\delta}} \cdot \delta_{yaw}$$
$$C_{D} = k \cdot (C_{N}^2 + C_{y}^2)$$

These coefficients are updated at every integrator step by reading the `current_deltas` from the controller state.
