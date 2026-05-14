# Plots and Analysis

The simulation generates a suite of analysis plots saved to the `results/YYYYMMDD_HHMMSS/plots/` directory.

## Directory Structure

- `simulation/`: Overall flight characteristics (full duration).
- `control/`: Detailed analysis of the active-control phase.

## Key Plots

### 1. Trajectory Tracking
Visualizes the real flight path against the reference trajectory in local ENU coordinates.
- **3D Trajectory**: Shows the spatial deviation.
- **Tracking Errors**: Plots the magnitude of the error vector $\|\vec{e}\|_2$ and per-axis errors.

### 2. Control Surface Activity
- **Fin Deflections**: Shows the history of the 4 control fins in degrees.
- **Saturation**: Highlights when fins reach the limit scheduled by dynamic pressure.

### 3. Vehicle State
- **Attitude (Euler)**: Roll, Pitch, and Yaw angles over time.
- **Body Rates**: Angular velocities $(\omega_x, \omega_y, \omega_z)$.
- **Dynamic Pressure ($q$)**: Indicates the aerodynamic authority available for control.

## Interpretation Guide

| Symptom | Probable Cause |
| :--- | :--- |
| **Steady-state position error** | Low integral gain or constant disturbance (wind). |
| **High-frequency fin vibration** | Excessive $K_d$ gain or integration noise. |
| **Divergent trajectory** | Controller delay too high or lack of aerodynamic authority. |
