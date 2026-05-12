# Plots and Analysis

The simulation generates a suite of analysis plots saved to the `results/<run_id>/plots/` directory. These are organized into two subdirectories to distinguish between overall flight characteristics and detailed control performance.

## Directory Structure

```
results/<run_id>/plots/
├── simulation/             # Full-flight trajectory and vehicle state
│   ├── trajectory_3d.png
│   ├── trajectory_2d_projections.png
│   ├── rocket.png          # Rocket static properties
│   ├── static_margin.png   # Stability analysis
│   └── motor_thrust.png    # Thrust profile
└── control/                # Active-control-phase detailed analysis
    ├── position_per_axis.png
    ├── tracking_errors.png
    ├── fin_actuation.png
    ├── attitude_euler.png
    ├── body_rates.png
    ├── trajectory_3d.png
    └── trajectory_2d_projections.png
```

## Key Plots

### 1. Trajectory 3D
Visualizes the real flight path against the reference trajectory in local ENU coordinates.
- **Solid Blue**: Real path.
- **Dashed Black**: Reference path.
- **Red X**: Apogee.

### 2. Tracking Errors
Analyzes the deviation from the reference during the active-control phase.
- **3D Norm**: $\| \vec{p}_{ref} - \vec{p} \|_2$
- **Per-Axis**: $e_x, e_y, e_z$ separately.

### 3. Fin Actuation
Shows the deflection history of the four control fins.
- **Units**: Degrees.
- **Limits**: Visualizes saturation at $\pm \delta_{limit}$.

### 4. Attitude and Rates
- **Euler Angles**: Roll, Pitch, and Yaw in the ZYX convention.
- **Body Rates**: Angular velocities $\omega_x, \omega_y, \omega_z$ in rad/s.

## Interpretation Guide

| Symptom | Possible Cause |
| :--- | :--- |
| **High Tracking Error** | Inadequate guidance gains or aggressive reference. |
| **Fin Saturation** | Gains too high or low aerodynamic authority. |
| **Oscillating Body Rates** | Excessive derivative gain ($K_d$) or control delay. |
| **Large Roll Angle** | Insufficient roll damping ($K_{p,roll}$). |
