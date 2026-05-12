# Module: `src/metrics.py`

## Overview

Computes **quantitative tracking performance metrics** for the controlled flight. Focuses on the active-control phase (when fins are actively commanding deflection) for meaningful performance evaluation.

Distinguishes between two windows:
- **Active-control window**: samples where the controller diagnostics report `control_active=True`. This is the authoritative signal, not nonzero deltas alone.
- **Ascent window**: from control activation to apogee (may include post-cutoff coasting when dynamic pressure drops).

## Key Functions

### `compute_tracking_metrics(flight_history, reference, config, controller_state=None)`

**Purpose**: Computes tracking errors and control performance metrics.

**Signature**:
```python
def compute_tracking_metrics(
    flight_history: list[dict],
    reference: dict,
    config: object,
    controller_state: dict = None
) -> dict:
```

**Workflow**:

1. **Identify Control Window**:
   ```python
   start_idx, end_idx = get_control_window_indices(flight_history)
   ctrl_history = flight_history[start_idx:end_idx+1]
   ```

2. **Compute Per-Timestep Errors** (in control phase):
   ```python
   for sample in ctrl_history:
       t = sample['time_s']
       ref = sample_reference(reference, t)
       
       # 3D position error
       e_vec = ref['position_enu_m'] - sample['position_enu_m']
       dist_error = np.linalg.norm(e_vec)
       
       errors_3d.append(dist_error)
       errors_x.append(e_vec[0])  # East error
       errors_y.append(e_vec[1])  # North error
       errors_z.append(e_vec[2])  # Up error
       deltas.append(sample['deltas'])
   ```

3. **Aggregate Metrics**:
   ```python
   metrics = {
       "ctrl_mae_3d_m": float(np.mean(errors_3d)),           # Mean Absolute Error
       "ctrl_rmse_3d_m": float(np.sqrt(np.mean(errors_3d**2))),  # Root Mean Square Error
       "ctrl_max_error_3d_m": float(np.max(errors_3d)),
       "ctrl_mae_x_m": float(np.mean(np.abs(errors_x))),
       "ctrl_mae_y_m": float(np.mean(np.abs(errors_y))),
       "ctrl_mae_z_m": float(np.mean(np.abs(errors_z))),
       "max_fin_deflection_deg": float(np.max(abs_deltas_deg)),
   }
   ```

 4. **Fin Saturation Ratio**:
    ```python
    # delta_max_rad is in controller_state (loaded from TOML, not config)
    if controller_state and "delta_max_rad" in controller_state:
        sat_mask = np.any(np.abs(deltas) >= 0.95 * controller_state["delta_max_rad"], axis=1)
        sat_count = np.sum(sat_mask)
        metrics["fin_saturation_ratio"] = float(sat_count / total_ctrl_samples)
    ```

5. **Flight Summary** (Full Flight):
   ```python
   times = np.array([s['time_s'] for s in flight_history])
   pos_local = np.array([s['position_enu_m'] for s in flight_history])
   pos_asl = np.array([s['position_asl_m'] for s in flight_history])
   speeds = np.linalg.norm([s['velocity_enu_m_s'] for s in flight_history], axis=1)
   
   apogee_idx = np.argmax(pos_local[:, 2])
   
   metrics["summary"] = {
       "launch_altitude_asl_m": float(pos_asl[0, 2]),
       "max_altitude_asl_m": float(pos_asl[apogee_idx, 2]),
       "max_altitude_local_m": float(pos_local[apogee_idx, 2]),
       "time_of_apogee_s": float(times[apogee_idx]),
       "final_time_s": float(times[-1]),
       "max_speed_m_s": float(np.max(speeds)),
       "control_phase_start_s": float(times[start_idx]),
       "control_phase_end_s": float(times[end_idx]),
       "control_phase_duration_s": float(times[end_idx] - times[start_idx]),
       "max_fin_deflection_deg": metrics["max_fin_deflection_deg"],
       "fin_saturation_ratio": metrics["fin_saturation_ratio"]
   }
   ```

**Returns**: Dict with `ctrl_*` metrics and `"summary"` sub-dict.

---

## Metrics Definitions

### Control Phase Metrics (prefix: `ctrl_`)

All computed only during the **control phase** (first fin deflection to apogee).

| Metric | Description | Units | Interpretation |
|--------|-------------|-------|----------------|
| `ctrl_mae_3d_m` | Mean Absolute 3D Error | m | Average distance from reference |
| `ctrl_rmse_3d_m` | Root Mean Square 3D Error | m | Penalizes large errors more |
| `ctrl_max_error_3d_m` | Maximum 3D Error | m | Worst-case deviation |
| `ctrl_mae_x_m` | MAE in East (X) | m | Lateral tracking (East) |
| `ctrl_mae_y_m` | MAE in North (Y) | m | Lateral tracking (North) |
| `ctrl_mae_z_m` | MAE in Up (Z) | m | Vertical tracking |
| `max_fin_deflection_deg` | Max fin deflection | deg | Control effort |
| `fin_saturation_ratio` | Saturation ratio | 0-1 | Fraction of time any fin is saturated |

### Flight Summary Metrics

| Metric | Description | Units |
|--------|-------------|-------|
| `launch_altitude_asl_m` | Launch site elevation | m |
| `max_altitude_asl_m` | Maximum altitude (ASL) | m |
| `max_altitude_local_m` | Maximum altitude (local ENU) | m |
| `time_of_apogee_s` | Time at apogee | s |
| `final_time_s` | Simulation end time | s |
| `max_speed_m_s` | Maximum velocity magnitude | m/s |
| `control_active_start_s` | First active-control sample time | s |
| `control_active_end_s` | Last active-control sample time | s |
| `control_active_duration_s` | Active-control window duration | s |
| `ascent_window_start_s` | Ascent window start (same as active start) | s |
| `ascent_window_end_s` | Ascent window end (apogee time) | s |
| `ascent_window_duration_s` | Ascent window duration | s |
| `max_control_cD` | Maximum control-induced drag coefficient | - |
| `mean_control_cD` | Mean control-induced drag coefficient | - |
| `duplicate_callback_count` | Number of duplicate controller callbacks detected | - |

---

## Output Formats

### JSON (`metrics.json`)

```json
{
    "ctrl_mae_3d_m": 5.23,
    "ctrl_rmse_3d_m": 6.81,
    "ctrl_max_error_3d_m": 12.45,
    "ctrl_mae_x_m": 2.10,
    "ctrl_mae_y_m": 1.85,
    "ctrl_mae_z_m": 4.92,
    "max_fin_deflection_deg": 12.5,
    "fin_saturation_ratio": 0.05,
    "summary": {
        "launch_altitude_asl_m": 1000.0,
        "max_altitude_asl_m": 2450.3,
        ...
    }
}
```

### CSV (`flight_summary.csv`)

Single-row CSV with columns matching `summary` keys.

---

## Dependencies

- `numpy`: Array operations, mean, max, etc.
- `src.utils`: `get_control_window_indices()`
- `src.reference`: `sample_reference()`

---

## Control Phase Detection

Two window functions in `utils.py` determine the metric scope:

### Active-Control Window
```python
# From controller diagnostics (authoritative):
active_times = [d["time_s"] for d in diag if d["control_active"]]
start = first active time
end = last active time

# Fallback (no diagnostics): nonzero deltas
ctrl_active_mask = np.any(np.abs(deltas) > 1e-6, axis=1)
start = first True index
end = last True index
```

### Ascent Window
```python
# From controller diagnostics or deltas (same start as active)
end = apogee (max altitude in Z)
```

The active-control window is used for control performance metrics.
The ascent window is used for apogee and flight summary metrics.

**Why separate?** Control may deactivate before apogee (e.g., low dynamic pressure cutoff). Using the ascent window for control metrics would dilute them with non-control coasting samples.

---

## Interpretation Guide

### Good Tracking
- `ctrl_mae_3d_m` < 5m
- `ctrl_rmse_3d_m` < 10m
- `fin_saturation_ratio` < 0.1 (fins not frequently saturated)
- `max_control_cD` < 1.0 (moderate control drag)

### Poor Tracking
- `ctrl_mae_3d_m` > 20m
- `ctrl_rmse_3d_m` > 30m
- `fin_saturation_ratio` > 0.5 (fins saturated >50% of the time)
- `max_control_cD` > 5.0 (excessive control drag)

### Possible Causes of Poor Tracking
1. **Gains too low**: Increase `Kp_guidance`, `Kp_attitude`
2. **Gains too high**: Decrease `Kp_attitude` (may cause oscillation)
3. **Reference infeasible**: Trajectory too aggressive (high accelerations)
4. **Control starts too late**: Decrease `control_start_delay_s`
5. **Fins saturated**: Increase `delta_max_rad` or reduce gains
6. **Excessive control drag**: Increase `qbar_full_authority_pa` or reduce `cN_delta_per_rad`
7. **Reference mismatch**: Ensure reference is compatible with launch configuration

---

## Caveats

1. **Control window detection**: Uses simple threshold (1e-6 rad). May not work correctly if fins are deflected but control is "off" (e.g., during motor burn with manual zeroing).

2. **Interpolation error**: `sample_reference()` uses linear interpolation. If reference is sparsely sampled, error computation may be inaccurate.

3. **3D norm**: The 3D error metric weights all axes equally. In reality, vertical (Z) error may be more/less important than lateral (X,Y) error.

4. **Saturation detection**: Uses 95% threshold (`0.95 * delta_max_rad`). Adjust if needed.

5. **No cross-coupling metrics**: Doesn't measure correlation between axes or coupling effects.

6. **Full-flight summary**: The `"summary"` section includes full-flight data (not just control phase). This is intentional for flight characterization.

---

## Example Usage

```python
from src.metrics import compute_tracking_metrics
import src.reference as ref_mod

# Assume flight_history and reference are available
metrics = compute_tracking_metrics(flight_history, reference, config)

# Print key metrics
print(f"Mean tracking error: {metrics['ctrl_mae_3d_m']:.2f} m")
print(f"Max error: {metrics['ctrl_max_error_3d_m']:.2f} m")
print(f"Fin saturation: {metrics['fin_saturation_ratio']*100:.1f}%")

# Access flight summary
summary = metrics['summary']
print(f"Apogee: {summary['max_altitude_local_m']:.1f} m at t={summary['time_of_apogee_s']:.1f}s")
print(f"Control phase: {summary['control_phase_duration_s']:.1f}s")
```
