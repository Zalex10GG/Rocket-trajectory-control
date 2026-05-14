# `tools/tunning.py`

## Purpose

Identifies the short-period pitch dynamics of the rocket near the maximum dynamic pressure (max-Q) and estimates initial PID gains for the attitude controller.

## Workflow

1.  **System Identification**: Applies a pulse or step input to the fins during an open-loop simulation to record the pitch response.
2.  **Model Fitting**: Fits a second-order transfer function to the observed $\theta$ (attitude) and $q$ (pitch rate) signals.
3.  **Gain Estimation**: Uses the Ziegler-Nichols reaction-curve method to suggest $K_p, K_i, K_d$ values.

## Mathematical Identification

The identified model follows the second-order form:

$$G(s) = \frac{\Theta(s)}{\Delta(s)} = \frac{K \omega_n^2}{s^2 + 2 \zeta \omega_n s + \omega_n^2}$$

Where:
- $K$: Static gain.
- $\zeta$: Damping ratio.
- $\omega_n$: Natural frequency.

## Command

```powershell
uv run py tools/tunning.py
```

## Outputs

All diagnostic files are saved in `tools/results/`:
- `pitch_identification.png`: Comparison between simulation data and the fitted model.
- `pitch_bode.png`: Magnitude and phase of the plant.
- `pitch_modes.png`: Pole-zero map showing damping guides.

The suggested gains are printed in the console and must be manually updated in `config.py` for use in the main simulation.
