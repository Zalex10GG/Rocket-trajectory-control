# Coeficientes Aerodinámicos para Aletas de Control (Solo Deflexión)

Este documento lista los coeficientes que se deben estimar para modelar **solo el incremento aerodinámico debido a la deflexión de las aletas traseras**, cuando ya se dispone de la aerodinámica pasiva mediante `TrapezoidalFins` (RocketPy).

## Modelo Híbrido Actual

- **Aerodinámica pasiva**: `TrapezoidalFins` (RocketPy nativo).
- **Incremento por control**: `GenericSurface` nombrada `"Control Fin Deflection Increment"`.

## Derivadas Requeridas (Solo Incremento)

Dado que la contribución total de una aleta es:

```text
C_total(alpha, beta, delta) = C_passive(alpha, beta) + C_incremental(alpha, beta, delta)
```

Y el incremento se define como:

```text
C_incremental(alpha, beta, delta) = C_total(alpha, beta, delta) - C_total(alpha, beta, 0)
```

Los coeficientes que necesitamos son las derivadas respecto a `delta` (deflexión), evaluadas en `delta = 0`:

| Coeficiente | Significado | Unidades | Fórmula | Cómo estimarlo |
|-------------|-------------|---------|---------|---------------|
| `dCN_ddelta` | Incremento de fuerza normal por deflexión (Body Y) | 1/rad | `∂CN_total / ∂delta \| delta=0` | Curva CN vs delta para alpha fijo (0) y beta (0), calcular pendiente en delta=0. |
| `dCY_ddelta` | Incremento de fuerza lateral por deflexión (Body X) | 1/rad | `∂CY_total / ∂delta \| delta=0` | Curva CY vs delta para alpha (0) y beta fijo (0), calcular pendiente en delta=0. |
| `dCD_ddelta` | Incremento de drag por deflexión | 1/rad | `∂CD_total / ∂delta \| delta=0` | Curva CD vs delta; suele ser pequeño para ángulos pequeños. |
| `k_drag_induced` | Factor de drag inducido (opcional) | - | `CD = CD_0 + k * (CN^2 + CY^2)` | Si CD incremental depende de la lift, estimar este término. |

## Coeficientes en TOML (`[control_actuation]`)

Estos valores deben ir en el TOML y ser leídos por `FinAdapter`:

```toml
[control_actuation]
reference_area_m2 = 0.017671458676442587
reference_length_m = 0.15

# Derivadas de incremento (solo control)
cN_delta_per_rad = 4.8        # dCN/ddelta
cy_delta_per_rad = 4.8        # dCY/ddelta
cd_delta_per_rad = 0.0        # dCD/ddelta (drag incremental pequeno)

# Momentos: dejamos que RocketPy calcule por brazo de palanca
cm_delta_per_rad = 0.0        # dCm/ddelta (momento local al CP, no total)
cn_moment_delta_per_rad = 0.0  # dCn/ddelta
cl_delta_per_rad = 0.5         # dCl/ddelta (rollo)
```

## Qué NO debemos dar (para evitar doble conteo)

- **No** dar `clalpha`, `cL_alpha`, `cD_alpha`, etc., en la `GenericSurface` de control.
- **No** dar `cm_alpha`, `cn_alpha`, `cl_alpha` en esa superficie.
- **No** dar `cm_delta` como momento total alrededor del CG: eso lo calcura RocketPy usando el CP de la `GenericSurface` y el CG del cohete.

## Resumen de Uso en `src/fin_model.py`

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
        Solo deflexión: cN_delta * delta_pitch
        """
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[1] - deltas[3]) / 2.0
        return -self.cN_delta * delta_pitch

    def cq_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        cQ (GenericSurface -> RocketPy aero frame) = Side force in Body X
        Solo deflexión: cy_delta * delta_yaw
        """
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[0] - deltas[2]) / 2.0
        return self.cy_delta * delta_yaw

    def cd_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return self.cd_delta  # Asumimos drag incremental constante / pequeno

    def cm_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return 0.0  # RocketPy calcula momento por brazo

    def cn_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        return 0.0

    def cl_roll_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        deltas = self.get_current_deltas()
        delta_roll = np.mean(deltas)
        return self.cl_delta * delta_roll
```

## Notas para la Estimación

1. **Túnel de viento o CFD**:
   - Generar curvas `CN(delta)` y `CY(delta)` para `alpha = 0`, `beta = 0`, Mach bajo.
   - La pendiente en `delta = 0` es `dCN/ddelta` y `dCY/ddelta`.

2. **Teórico (lámina plana simplificada)**:
   - Para aletas pequeñas en flujo incompresible: `dCN/ddelta ≈ 2 * (área_aleta / área_referencia) * K` (factor de evaluación 3D).
   - Puedes usar la derivada de paso (`cN_alpha`) y escalarla por la geometría de la aleta.

3. **Drag inducido**:
   - `k_drag_induced` en TOML puede estimarse si el drag incremental escala con `CN^2 + CY^2`.

4. **Momentos**:
   - Si la superficie `GenericSurface` está colocada exactamente en el CP de las aletas, el momento local es cercano a cero.
   - RocketPy calculará automáticamente: `Moment = (CP - CG) x Force`.
