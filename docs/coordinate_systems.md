# Coordinate Systems and Frame Conventions

## Overview

This document describes the coordinate systems used in the Rocket Control TFG project. The primary reference frame for navigation and control is the **local ENU (East-North-Up)** frame.

## Local ENU Frame

The ENU frame is a local tangent plane system defined at the launch site:
- **Origin ($O_{ENU}$)**: Launch pad position.
- **X-axis ($E$)**: Points East.
- **Y-axis ($N$)**: Points North.
- **Z-axis ($U$)**: Points Up (away from Earth's center).

The relationship between the absolute position $\vec{p}_{abs}$ (from RocketPy's geodetic solver) and the local position $\vec{p}_{local}$ is:

$$\vec{p}_{local} = \vec{p}_{abs} - \vec{p}_{launch}$$

## Rocket Body Frame

The body frame is attached to the rocket:
- **X-axis ($x_b$)**: Transverse axis (pointing towards Fin 1).
- **Y-axis ($y_b$)**: Transverse axis (pointing towards Fin 2).
- **Z-axis ($z_b$)**: Longitudinal axis (pointing from tail to nose).

## Attitude Representation

### 1. Quaternions

Attitude is represented by a unit quaternion $q$ in scalar-first format $[w, x, y, z]$, defining the rotation from the ENU frame to the Body frame.

$$q = \begin{bmatrix} \cos(\theta/2) \\ \hat{n} \sin(\theta/2) \end{bmatrix}$$

where $\theta$ is the rotation angle and $\hat{n}$ is the unit rotation axis.

### 2. Attitude Error

The error between the desired orientation $q_{ref}$ and the current orientation $q$ is:

$$q_e = q_{ref} \otimes q^*$$

This error quaternion represents the rotation needed to bring the rocket from its current orientation to the reference.

### 3. Euler Angles

For visualization and human-readable analysis, quaternions are converted to ZYX Euler angles (Roll $\phi$, Pitch $\theta$, Yaw $\psi$):

- **Roll ($\phi$)**: Rotation around $x_b$.
- **Pitch ($\theta$)**: Rotation around $y_b$.
- **Yaw ($\psi$)**: Rotation around $z_b$.

## Control Surface Numbering

Fins are arranged in a cross (+) configuration and numbered by their angular position in the body $x_y$ plane:

| Fin | Angle | Position |
| :--- | :--- | :--- |
| **Fin 1** | $0^\circ$ | Right ($+x_b$) |
| **Fin 2** | $90^\circ$ | Top ($+y_b$) |
| **Fin 3** | $180^\circ$ | Left ($-x_b$) |
| **Fin 4** | $270^\circ$ | Bottom ($-y_b$) |

## Summary Table

| Quantity | Symbol | Frame | Units |
| :--- | :--- | :--- | :--- |
| Position | $\vec{p}$ | ENU | m |
| Velocity | $\vec{v}$ | ENU | m/s |
| Acceleration | $\vec{a}$ | ENU | m/s² |
| Attitude | $q$ | ENU $\to$ Body | - |
| Angular Rate | $\vec{\omega}$ | Body | rad/s |
| Deflection | $\delta$ | Body | rad |
