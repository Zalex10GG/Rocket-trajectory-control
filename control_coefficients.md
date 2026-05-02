# Control Aerodynamic Coefficients for Fin Deflection

This document lists the coefficients to estimate for modeling the aerodynamic increment due to rear fin deflection, when passive aerodynamics are already provided via `TrapezoidalFins` (RocketPy).

## Current Hybrid Model

- **Passive aerodynamics**: `TrapezoidalFins` (native RocketPy).
- **Control increment**: `GenericSurface` named `"Control Fin Deflection Increment"`.

## Required Derivatives (Increment Only)

Since the total contribution of a fin is:

```text
C_total(alpha, beta, delta) = C_passive(alpha, beta) + C_incremental(alpha, beta, delta)
```

And the increment is defined as:

```text
C_incremental(alpha, beta, delta) = C_total(alpha, beta, delta) - C_total(alpha, beta, 0)
```

The coefficients needed are the derivatives with respect to `delta` (deflection), evaluated at `delta = 0`:

| Coefficient | Meaning | Units | Formula | How to estimate |
|-------------|---------|---------|---------|---------------|
| `dCN_ddelta` | Normal force increment due to deflection (Body Y) | 1/rad | `∂CN_total / ∂delta \| delta=0` | CN vs delta curve for fixed alpha (0) and beta (0), compute slope at delta=0. |
| `dCY_ddelta` | Lateral force increment due to deflection (Body X) | 1/rad | `∂CY_total / ∂delta \| delta=0` | CY vs delta curve for alpha (0) and fixed beta (0), compute slope at delta=0. |
| `dCD_ddelta` | Drag increment due to deflection | 1/rad | `∂CD_total / ∂delta \| delta=0` | CD vs delta curve; usually small for small angles. |
| `k_drag_induced` | Induced drag factor (optional) | - | `CD = CD_0 + k * (CN^2 + CY^2)` | If incremental CD depends on lift, estimate this term. |

## Coefficients in TOML (`[control_actuation]`)

These values must be in the TOML and read by `FinAdapter`:

```toml
[control_actuation]
reference_area_m2 = 0.007853981633974483
reference_length_m = 0.1452

# Increment derivatives (control only)
cN_delta_per_rad = 9.343586365106        # dCN/ddelta
cy_delta_per_rad = 9.343586365106        # dCY/ddelta
cd_delta_per_rad = 0.0        # dCD/ddelta (small incremental drag)

# Moments: let RocketPy compute via moment arm
cm_delta_per_rad = 0.0        # dCm/ddelta (local moment at CP, not total)
cn_moment_delta_per_rad = 0.0  # dCn/ddelta
cl_delta_per_rad = 0.5         # dCl/ddelta (roll)
```

## What NOT to Provide (to avoid double counting)

- **Do not** provide `clalpha`, `cL_alpha`, `cD_alpha`, etc., in the control `GenericSurface`.
- **Do not** provide `cm_alpha`, `cn_alpha`, `cl_alpha` in that surface.
- **Do not** provide `cm_delta` as total moment around the CG: RocketPy will compute it using the `GenericSurface` CP and the rocket CG.

## Usage Summary in `src/fin_model.py`

```python
class FinAdapter:
    def __init__(self, controller_state, actuation_params):
        self.cN_delta = actuation_params.get("cN_delta_per_rad", 0.0)
        self.cy_delta = actuation_params.get("cy_delta_per_rad", 0.0)
        self.cd_delta = actuation_params.get("cd_delta_per_rad", 0.0)
        self.cl_delta = actuation_params.get("cl_delta_per_rad", 0.0)

        # NO passive terms here; they are in TrapezoidalFins

    def cl_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        cL (GenericSurface -> RocketPy aero frame) = - (Normal force in Body Y)
        Deflection only: cN_delta * delta_pitch
        """
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[1] - deltas[3]) / 2.0
        return -self.cN_delta * delta_pitch

    def cq_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        cQ (GenericSurface -> RocketPy aero frame) = Side force in Body X
        Deflection only: cy_delta * delta_yaw
        """
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[0] - deltas[2]) / 2.0
        return self.cy_delta * delta_yaw

    def cd_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return self.cd_delta  # Assume constant/small incremental drag

    def cm_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return 0.0  # RocketPy computes moment via arm

    def cn_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return 0.0

    def cl_roll_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        deltas = self.get_current_deltas()
        delta_roll = np.mean(deltas)
        return self.cl_delta * delta_roll
```

## Estimation Notes

1. **Wind tunnel or CFD**:
   - Generate `CN(delta)` and `CY(delta)` curves for `alpha = 0`, `beta = 0`, low Mach.
   - The slope at `delta = 0` is `dCN/ddelta` and `dCY/ddelta`.

2. **Theoretical (simplified flat plate)**:
   - For small fins in incompressible flow: `dCN/ddelta ≈ 2 * (fin_area / reference_area) * K` (3D scaling factor).
   - You can use the pitch derivative (`cN_alpha`) and scale it by fin geometry.

3. **Induced drag**:
   - `k_drag_induced` in TOML can be estimated if incremental drag scales with `CN^2 + CY^2`.

4. **Moments**:
   - If the `GenericSurface` is placed exactly at the fin CP, the local moment is near zero.
   - RocketPy will automatically compute: `Moment = (CP - CG) x Force`.
