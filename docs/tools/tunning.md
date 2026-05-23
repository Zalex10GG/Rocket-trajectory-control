# `tools/tunning.py`

## Purpose

Identifies the short-period pitch and roll dynamics of the rocket near the maximum dynamic pressure (max-Q, $t \approx 2.13$ s) and automatically estimates initial PID gains for the attitude controller using Ziegler-Nichols tuning methods.

## Workflow

1.  **System Identification**: Runs open-loop simulations applying a pulse or step input to the tail fins at max-Q to record the decoupled dynamic response.
2.  **Model Fitting**:
    - **Pitch System**: Fits a second-order underdamped transfer function to the pitch angle ($\theta$) and pitch rate ($\omega_x$) response.
    - **Roll System**: Fits a first-order lag model to the roll rate ($\omega_z$) response.
3.  **Gain Estimation**: Uses the Ziegler-Nichols reaction-curve / transient-response formulas to calculate the suggested PID/PI/P control gains.

## Mathematical Identification

### 1. Pitch Dynamics (2nd-Order Model)

The pitch plant is identified as a second-order transfer function relating pitch attitude $\Theta(s)$ to fin deflection command $\Delta(s)$:

$$G_{pitch}(s) = \frac{\Theta(s)}{\Delta(s)} = \frac{K_p \omega_n^2}{s^2 + 2 \zeta \omega_n s + \omega_n^2}$$

Where:
- $K_p$: Static plant gain.
- $\zeta$: Damping ratio (captures aerodynamic damping).
- $\omega_n$: Natural frequency (captures the aerodynamic stiffness / restoring moment).

**Ziegler-Nichols Reaction Curve for Pitch:**
Using the step response delay $L$ (dead time) and slope $R$, the PID gains are computed as:
- $K_p = \frac{1.2}{R \cdot L}$
- $K_i = \frac{1}{2.0 \cdot L}$
- $K_d = 0.5 \cdot L$

### 2. Roll Dynamics (1st-Order Lag Model)

The roll plant is modeled directly in terms of angular rate $\Omega_z(s)$ vs roll command $\Delta_r(s)$:

$$G_{roll}(s) = \frac{\Omega_z(s)}{\Delta_r(s)} = \frac{K_r}{\tau s + 1}$$

Where:
- $K_r$: Static roll gain.
- $\tau$: Time constant of the boundary layer and roll acceleration.

For a first-order lag with delay $L$ and time constant $\tau$, Ziegler-Nichols suggests PI gains:
- $K_p = \frac{0.9 \cdot \tau}{K_r \cdot L}$
- $K_i = 0.27 \cdot \frac{K_p}{\tau}$
- $K_d = 0.0$

## Command

Run the auto-tuning tool:
```powershell
uv run py tools/tunning.py
```

## Outputs

All diagnostic and fitted curves are saved in `tools/results/`:
- `pitch_identification.png`: Pitch angle and rate vs fitted second-order model. Shows the high-fidelity simultaneous fit of attitude ($\theta$) and angular rate ($q$).
- `pitch_bode.png`: Frequency response (Bode plot) showing gain and phase margins.
- `pitch_modes.png`: Pole-zero map of identified pitch poles.
  - **Grid Annotations**: Features highly precise grid lines indicating constant damping ratios ($\zeta = 0.5, 0.707, 0.9$) plotted via $\theta_{\text{polar}} = \pi - \arccos(\zeta)$ and constant natural frequency circles ($\omega_n = 10, 20, 30, 40, 50$ rad/s).
  - **Geometric Integrity**: Plotted with a strict 1:1 aspect ratio (`equal`) to prevent geometric distortion of constant frequency circles and damping angles.
- `roll_identification.png`: Roll rate response vs fitted first-order lag model.
- `roll_bode.png` and `roll_modes.png`: Frequency domain characteristics and pole-zero map of the roll plant.
  - **Poles Plotted**:
    - **Roll Damping Pole** at $s = -1/\tau \approx -200.0$ rad/s, representing the high-speed roll rate boundary layer lag.
    - **Integrator / Attitude Pole** at $s = 0$ rad/s, representing the physical integration from roll rate $\omega_z$ to roll angle $\phi$, corresponding to the attitude transfer function $G_{\phi}(s) = \frac{K_r}{s(\tau s + 1)}$.

The suggested pitch/yaw gains are printed directly to the console as Ziegler-Nichols baseline values. In the current configuration, those baseline values correspond to `Kp_attitude_zn`, `Ki_attitude_zn`, and `Kd_attitude_zn`; the active pitch/yaw gains are those baselines multiplied by `attitude_gain_scale`.


