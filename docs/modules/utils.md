# Module: `src/utils.py`

## Overview

Provides utility functions for quaternion mathematics, Euler angle conversions, and simulation window detection.

## Quaternion Mathematics

Quaternions are represented as 4-element vectors in scalar-first format: $q = [w, x, y, z]$.

### 1. Conjugate

$$q^* = [w, -x, -y, -z]$$

### 2. Multiplication (Hamilton Product)

Given $q_1$ and $q_2$:

$$q_1 \otimes q_2 = \begin{bmatrix} w_1 w_2 - x_1 x_2 - y_1 y_2 - z_1 z_2 \\ w_1 x_2 + x_1 w_2 + y_1 z_2 - z_1 y_2 \\ w_1 y_2 - x_1 z_2 + y_1 w_2 + z_1 x_2 \\ w_1 z_2 + x_1 y_2 - y_1 x_2 + z_1 w_2 \end{bmatrix}$$

### 3. Rotation from Vector $\vec{v}_1$ to $\vec{v}_2$

Computes the shortest-path quaternion $q$ such that $R(q)\vec{v}_1 = \vec{v}_2$. Both vectors must be normalized.

$$\vec{a} = \vec{v}_1 \times \vec{v}_2$$
$$s = \sqrt{2(1 + \vec{v}_1 \cdot \vec{v}_2)}$$
$$q = \left[ \frac{s}{2}, \frac{a_x}{s}, \frac{a_y}{s}, \frac{a_z}{s} \right]$$

Special handling is implemented for nearly identical or opposite vectors to avoid numerical singularities.

## Euler Angles

Converts $q = [w, x, y, z]$ to ZYX Euler angles (Roll $\phi$, Pitch $\theta$, Yaw $\psi$) in radians.

$$\begin{aligned} \phi &= \operatorname{atan2}(2(wx + yz), 1 - 2(x^2 + y^2)) \\ \theta &= \operatorname{asin}(2(wy - zx)) \\ \psi &= \operatorname{atan2}(2(wz + xy), 1 - 2(y^2 + z^2)) \end{aligned}$$

## Window Detection

Functions to identify critical phases of flight from history:

- **Control Window**: Start and end indices where the controller was active.
- **Ascent Window**: Start of control until maximum altitude (apogee).

These functions prioritize data from the controller's internal diagnostic log for high precision.
