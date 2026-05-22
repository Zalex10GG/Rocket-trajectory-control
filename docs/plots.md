# Plots and Analysis

The simulation generates a comprehensive suite of 9 analysis plots saved to the timestamped directory: `results/YYYYMMDD_HHMMSS/plots/`.

## Directory Structure

The plots are organized into two subdirectories based on their time scope:
- **`simulation/`**: Overall flight characteristics across the entire flight duration (launch to apogee/landing).
- **`control/`**: Detailed diagnostics focused exclusively on the active control phase (from safety margin rail clearance to control termination).

---

## Detailed Plot Specifications

### 1. `trajectory_3d.png` (in both `simulation/` and `control/`)
A 3D spatial plot showing the actual 3-DOF trajectory of the rocket against the loaded reference trajectory in the local tangent ENU frame.
- **Axes**: East ($X$) vs. North ($Y$) vs. Up ($Z$) in meters.
- **Purpose**: Evaluates absolute lateral and vertical deviations.

### 2. `trajectory_2d_projections.png` (in both `simulation/` and `control/`)
A 3-panel 2D layout showing the orthographic projections of the flight path:
- **XY Plane**: Lateral drift (East vs. North).
- **XZ Plane**: Vertical profile from East (East vs. Altitude).
- **YZ Plane**: Vertical profile from North (North vs. Altitude).
- **Purpose**: Helps isolate drift and cross-wind impact.

### 3. `position_per_axis.png` (in `control/`)
A 3-panel time-series tracking positions per axis ($X_{local}, Y_{local}, Z_{local}$).
- **Legend Alignment**: Customized for optimal visibility without overlapping the data lines:
  - **X Axis (East)**: Bottom-Left legend.
  - **Y Axis (North)**: Top-Left legend.
  - **Z Axis (Up)**: Top-Left legend.
- **Purpose**: Analyzes tracking convergence and transient responses per axis.

### 4. `tracking_errors.png` (in `control/`)
Plots the magnitude of the 3D position tracking error vector $\|\vec{e}_{3D}\|_2$ and the individual per-axis error components over time.
- **Axes**: Error in meters vs. Time in seconds.
- **Purpose**: Pinpoints the exact moments of maximum trajectory deviation.

### 5. `fin_actuation.png` (in `control/`)
A high-fidelity diagnostic showing the deflections of all four control surfaces ($\delta_1, \delta_2, \delta_3, \delta_4$) in degrees over time.
- **Dynamic Clamping Bounds**: Overlays the live scheduled deflection limit bounds ($\pm\delta_{limit}(q)$) as dashed lines.
- **Purpose**: Instantly identifies control saturation, rate-limiting, and gain scheduling transitions.

### 6. `attitude_euler.png` (in `control/`)
Plots the achieved (obtained) Euler angles of the vehicle alongside the reference (required) Euler angles commanded by the guidance loop.
- **Axes**: Rotation angle in degrees vs. Time in seconds.
- **Conventions**: Decoupled pitch (rotation around Body $X$), yaw (rotation around Body $Y$), and roll (rotation around Body $Z$).
- **Purpose**: Directly assesses the attitude loop's tracking quality and phase lag.

### 7. `body_rates.png` (in `control/`)
Plots the angular velocities of the vehicle in the body-fixed axes:
- **$\omega_x$**: Pitch rate in degrees/sec.
- **$\omega_y$**: Yaw rate in degrees/sec.
- **$\omega_z$**: Roll rate in degrees/sec.
- **Purpose**: Evaluates angular damping and high-frequency structural vibration.

### 8. `velocity_per_axis.png` (in both `simulation/` and `control/`)
A 3-panel plot tracking the per-axis linear velocities ($V_x, V_y, V_z$) against the reference velocities.
- **Legend Alignment**:
  - **X Axis**: Bottom-Right legend for full simulation; Top-Right for the control phase.
  - **Y and Z Axes**: Top-Right legend.
- **Purpose**: Analyzes kinetic energy profile and velocity tracking performance.

### 9. `cd_vs_mach.png` (in `simulation/`)
Plots the base drag coefficient ($C_d$) of the rocket as a function of the Mach number.
- **X Limit**: Dynamic axis locked strictly to the maximum Mach number achieved during flight (instead of a fixed Mach 3 limit).
- **Marker**: Highlights the exact point of Maximum Flight Mach.
- **Purpose**: Illustrates drag transitions across subsonic, transonic, and supersonic regimes.

### 10. `gain_evolution.png` (in `control/`)
A dual-panel plot illustrating the dynamic scaling of control gains in response to changing atmospheric conditions.
- **Upper Panel**: Tracks Dynamic Pressure $q$ on the left axis (solid blue) and the dimensionless gain scaling factor $q_{scale}$ on the right axis (dashed orange). Shows the exact alignment of $q_{scale}=1.0$ at Max-Q ($t \approx 2.13$ s) and its progression up to the maximum scheduled cap as the dynamic pressure drops.
- **Lower Panel**: Tracks the actual active proportional gains ($K_{p,attitude}$ and $K_{p,roll}$) over time.
- **Purpose**: Validates the performance of the Gain Scheduling system, showing how the controller dynamically increases authority as the air gets thinner to prevent tracking degradation.

---

## Interpretation Guide

| Symptom | Probable Cause | Corrective Action |
| :--- | :--- | :--- |
| **Steady-state position error** | Low integral gain or extreme constant wind. | Increase $K_{i,att}$ or increase wind feedforward $K_{wind\_comp}$. |
| **High-frequency fin vibration** | Excessive $K_d$ gain or integration steps mismatch. | Reduce $K_{d,att}$ or increase actuator Command Filter Tau $\tau_s$. |
| **Fins pinned at limit bounds** | Aerodynamic control authority is overpowered by passive stability (stiffness). | Review passive stability margin (Leon 2's passive tail fins are too large). |
| **Aggressive attitude tracking delay** | Large guidance commands. | Adjust acceleration filter $\alpha_f$ or clip guidance correction. |

