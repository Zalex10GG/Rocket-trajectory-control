# Coordinate Systems and Frame Conventions

## Overview

This document describes the coordinate systems used in the Rocket Control TFG project. The primary reference frame for navigation and control is the **local ENU (East-North-Up)** frame.

## Local ENU Frame

The ENU frame is a local tangent plane system defined at the launch site:
- **Origin ($O_{ENU}$)**: Launch pad position.
- **X-axis ($E$)**: Points East.
- **Y-axis ($N$)**: Points North.
- **Z-axis ($U$)**: Points Up.

The relationship between the absolute position $\vec{p}_{abs}$ and the local position $\vec{p}_{local}$ is:

$$\vec{p}_{local} = \vec{p}_{abs} - \vec{p}_{launch}$$

## Rocket Body Frame (RocketPy Convention)

The body frame is fixed to the rocket structure. RocketPy uses a right-handed frame with the longitudinal axis along $z$:
- **X-axis ($x_b$)**: Points **Right** (starboard). This is the **pitch axis**.
- **Y-axis ($y_b$)**: Points **Down** (belly). This is the **yaw axis**.
- **Z-axis ($z_b$)**: Longitudinal, points **Tail to Nose** (forward). This is the **roll axis**.

## Attitude Representation

### 1. Quaternions
Attitude is represented by a unit quaternion $q$ in scalar-first format $[w, x, y, z]$, defining the rotation from the ENU frame to the Body frame.

### 2. Attitude Error
The error between the desired orientation $q_{ref}$ and the current orientation $q$ is computed using the quaternion conjugate:

$$q_e = q_{ref} \otimes q^*$$

The error vector components are mapped to control axes as:
- $q_{e,x}$ → **pitch error** (rotation around $x_b$)
- $q_{e,y}$ → **yaw error** (rotation around $y_b$)
- $q_{e,z}$ → **roll error** (rotation around $z_b$)

### 3. Euler Angles
For analysis, quaternions are converted to ZYX Euler angles:
- **Roll ($\phi$)**: Rotation around $z_b$ (longitudinal / nose axis).
- **Pitch ($\theta$)**: Rotation around $x_b$ (right / starboard axis).
- **Yaw ($\psi$)**: Rotation around $y_b$ (down / belly axis).

## Control Surface Numbering

Fins are arranged in a cruciform (+) configuration. The mixer maps virtual control commands `(pitch, yaw, roll)` to four fin deflections:

$$\begin{aligned}
\delta_0 &= pitch + roll \\
\delta_1 &= yaw + roll \\
\delta_2 &= -pitch + roll \\
\delta_3 &= -yaw + roll
\end{aligned}$$

| Fin index | Position | Control Axis |
| :--- | :--- | :--- |
| **0** | Right ($+x_b$) | **Pitch** / Roll |
| **1** | Down ($+y_b$) | **Yaw** / Roll |
| **2** | Left ($-x_b$) | **Pitch** / Roll |
| **3** | Up ($-y_b$) | **Yaw** / Roll |

The aerodynamic force extraction in `FinAdapter` mirrors this mapping:

$$\begin{aligned}
\Delta_{pitch} &= (\delta_0 - \delta_2) / 2 \\
\Delta_{yaw}   &= (\delta_1 - \delta_3) / 2 \\
\Delta_{roll}  &= \text{mean}(\delta_0, \delta_1, \delta_2, \delta_3)
\end{aligned}$$

## Summary Table

| Quantity | Symbol | Frame | Units |
| :--- | :--- | :--- | :--- |
| Position | $\vec{p}$ | ENU | m |
| Velocity | $\vec{v}$ | ENU | m/s |
| Acceleration | $\vec{a}$ | ENU | m/s² |
| Attitude | $q$ | ENU $\to$ Body | - |
| Angular Rate | $\vec{\omega}$ | Body | rad/s |
| Deflection | $\delta$ | Body | rad |
