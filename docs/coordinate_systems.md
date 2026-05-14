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

## Rocket Body Frame

The body frame is fixed to the rocket structure:
- **X-axis ($x_b$)**: Transverse axis.
- **Y-axis ($y_b$)**: Transverse axis.
- **Z-axis ($z_b$)**: Longitudinal axis, pointing from **Tail to Nose**.

## Attitude Representation

### 1. Quaternions
Attitude is represented by a unit quaternion $q$ in scalar-first format $[w, x, y, z]$, defining the rotation from the ENU frame to the Body frame.

### 2. Attitude Error
The error between the desired orientation $q_{ref}$ and the current orientation $q$ is computed using the quaternion conjugate:

$$q_e = q_{ref} \otimes q^*$$

### 3. Euler Angles
For analysis, quaternions are converted to ZYX Euler angles:
- **Roll ($\phi$)**: Rotation around $x_b$.
- **Pitch ($\theta$)**: Rotation around $y_b$.
- **Yaw ($\psi$)**: Rotation around $z_b$.

## Control Surface Numbering

Fins are arranged in a cross (+) configuration:

| Fin | Position | Control Axis |
| :--- | :--- | :--- |
| **Fin 1** | Right ($+x_b$) | Yaw / Roll |
| **Fin 2** | Top ($+y_b$) | Pitch / Roll |
| **Fin 3** | Left ($-x_b$) | Yaw / Roll |
| **Fin 4** | Bottom ($-y_b$) | Pitch / Roll |

## Summary Table

| Quantity | Symbol | Frame | Units |
| :--- | :--- | :--- | :--- |
| Position | $\vec{p}$ | ENU | m |
| Velocity | $\vec{v}$ | ENU | m/s |
| Acceleration | $\vec{a}$ | ENU | m/s² |
| Attitude | $q$ | ENU $\to$ Body | - |
| Angular Rate | $\vec{\omega}$ | Body | rad/s |
| Deflection | $\delta$ | Body | rad |
