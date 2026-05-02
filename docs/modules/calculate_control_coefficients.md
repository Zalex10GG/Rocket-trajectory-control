# Module: `calculate_control_coefficients.py`

## Overview

Standalone script to compute the incremental aerodynamic coefficients for fin deflection control using the Diederich theory and body-fin interference factors. It reads rocket geometry from a TOML file, calculates the coefficients, and updates the `[control_actuation]` section in-place while preserving comments and formatting.

## CLI Usage

```bash
# Dry run (print only)
uv run calculate_control_coefficients.py --dry-run

# Update coefficients using default Leon 2 TOML
uv run calculate_control_coefficients.py

# Specify custom TOML, Mach number and Oswald efficiency
uv run calculate_control_coefficients.py --toml data/rockets/custom.toml --mach 0.3 --oswald-efficiency 0.75
```

### Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--toml` | `data/rockets/leon_2.toml` | Path to the rocket TOML file |
| `--mach` | `0.0` | Design Mach number (0.0 to <1.0) |
| `--oswald-efficiency` | `0.7` | Oswald efficiency factor (e) |
| `--dry-run` | `False` | Print results without modifying the file |
| `--precision` | `12` | Decimal precision for rounding |

## Core Functions

### `calculate_fin_area(root_chord, tip_chord, span) -> float`

Calculates the trapezoidal fin area:

```python
A_fin = 0.5 * (root_chord + tip_chord) * span
```

### `calculate_midchord_sweep(root_chord, tip_chord, span, sweep_length) -> float`

Calculates the midchord sweep angle (Gamma_c) from geometry:

```python
delta_x_midchord = sweep_length + 0.5 * tip_chord - 0.5 * root_chord
Gamma_c = atan2(delta_x_midchord, span)
```

### `calculate_beta(mach) -> float`

Calculates the Prandtl-Glauert compressibility factor:

```python
beta = sqrt(1 - mach**2)
```

### `calculate_cn_delta_isolated(span, a_ref, a_fin, beta, gamma_c) -> float`

Calculates the normal force slope for a single isolated fin using the Diederich theory:

```python
cos_gamma_c = cos(gamma_c)
term_root = sqrt(1 + (beta * span**2 / (a_fin * cos_gamma_c))**2)
cn_delta_single = (2 * pi * span**2 / a_ref) / (1 + term_root)
```

### `calculate_interference_ktb(radius_body, span) -> float`

Calculates the body-fin interference factor:

```python
K_TB = 1 + radius_body / (span + radius_body)
```

### `calculate_k_drag_induced(a_fin, span, oswald_efficiency) -> float`

Calculates the induced drag factor k based on effective aspect ratio:

```python
# AR = 2 * span**2 / A_fin
k = 1 / (pi * AR * e)
k = A_fin / (2 * pi * span**2 * e)
```

## Workflow

1. **Read** TOML file to extract body radius, fin geometry and current `[control_actuation]` values.
2. **Calculate** fin area (A_fin), reference area (A_ref), midchord sweep (Gamma_c), compressibility (beta).
3. **Compute** isolated fin slope (cn_delta_single) using Diederich, apply interference (K_TB).
4. **Scale** to cruciform configuration: `cN_delta = 2 * cn_delta_single * K_TB`.
5. **Compute** induced drag factor (k_drag_induced) using Oswald efficiency.
6. **Update** only the required keys in `[control_actuation]` preserving comments and formatting.

## TOML Keys Updated

| Key | Description |
|-----|-------------|
| `cN_delta_per_rad` | Normal force increment derivative (1/rad) |
| `cy_delta_per_rad` | Lateral force increment derivative (1/rad) |
| `k_drag_induced` | Induced drag factor (dimensionless) |

## Notes

- `cd_delta_per_rad`, `cm_delta_per_rad` and `cn_moment_delta_per_rad` are not computed because:
  - Linear drag increment at delta=0 is zero.
  - Moments are computed by RocketPy using the CP-to-CG moment arm.
- The script preserves all comments and formatting of the original TOML file.
- For small fins in incompressible flow, the Diederich method provides a reliable semi-empirical estimate consistent with OpenRocket references.
