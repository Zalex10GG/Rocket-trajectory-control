# Module: `src/metrics.py`

## Overview

`src.metrics.compute_tracking_metrics()` computes tracking, saturation, control-drag, and flight-summary metrics from `flight_history`.

The nominal metrics focus on the active-control window. The active-control window is detected from controller diagnostics when available, with fin-deflection fallback logic in `src.utils`.

## Position Error

For each active-control sample:

```text
e_pos = reference position - simulated position
```

The function computes:

- `ctrl_mae_3d_m`
- `ctrl_rmse_3d_m`
- `ctrl_max_error_3d_m`
- `ctrl_mae_x_m`
- `ctrl_mae_y_m`
- `ctrl_mae_z_m`
- `ctrl_rmse_lateral_m`
- `ctrl_max_lateral_m`

## Fin And Saturation Metrics

The metrics include:

- `max_fin_deflection_deg`
- `fin_saturation_ratio`
- `saturation_time_s`
- `active_control_duration_s`
- `saturation_time_ratio`

When controller diagnostics exist, saturation is computed from `limited_deltas_rad` and the live `delta_limit_rad`.

## Control Diagnostics

When controller diagnostics exist, the metrics include:

- `max_control_cD`
- `mean_control_cD`
- `max_commanded_aoa_deg`
- `mean_commanded_aoa_deg`
- `duplicate_callback_count`
- last dynamic pressure
- last airspeed
- last wind speed

## Flight Summary

The `summary` object contains:

- launch altitude ASL
- maximum altitude ASL
- maximum local altitude
- time of apogee
- final simulation time
- maximum speed
- active-control start, end, and duration
- ascent-window start, end, and duration
- maximum fin deflection
- saturation ratio
- lateral RMSE
- control drag diagnostics
- commanded angle-of-attack diagnostics
- duplicate callback count

`src.simulation.export_results()` writes this summary to `flight_summary.csv`.

## Gain Sweep Metrics

`tools/sweep_gain_scale.py` computes its own apogee-limited metrics over the simulated ascent up to apogee. Those sweep metrics are exported to `tools/results/sweep/sweep_metrics.csv` and are documented in `docs/tools/sweep_gain_scale.md`.
