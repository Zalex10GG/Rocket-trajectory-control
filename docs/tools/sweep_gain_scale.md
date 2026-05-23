# `tools/sweep_gain_scale.py`

## Purpose

Sweeps `config.attitude_gain_scale` and evaluates trajectory tracking up to apogee. The tool is used to compare gain factors with consistent apogee-limited metrics and plots.

## Command

```bash
uv run py tools/sweep_gain_scale.py
```

## Sweep Configuration

Edit the constants at the top of the file:

```python
SCALE_MIN = 1.0
SCALE_MAX = 7.0
SCALE_STEP = 0.5
```

The interval is inclusive. If the step does not land exactly on `SCALE_MAX`, the endpoint is appended.

## Workflow

For each gain scale:

1. Load the nominal config and case data.
2. Set `config.save_results = False`.
3. Set `config.show_plots = False`.
4. Set `config.terminate_on_apogee = True`.
5. Set `config.attitude_gain_scale`.
6. Rebuild the controller.
7. Rebuild the rocket so controller state and fin adapter are fresh.
8. Run the closed-loop flight.
9. Compute metrics over the simulated history up to apogee.
10. Append the row to the sweep CSV.

## Metrics

`sweep_metrics.csv` contains:

- `gain_scale`
- `kp_attitude`, `ki_attitude`, `kd_attitude`
- `mean_3d_error_m`
- `max_3d_error_m`
- `max_lateral_error_m`
- `apogee_3d_error_m`
- `apogee_lateral_error_m`
- `apogee_height_error_m`
- `apogee_altitude_real_m`
- `reference_max_altitude_m`
- `reference_altitude_at_apogee_time_m`
- `apogee_time_s`
- `max_fin_deflection_deg`

`apogee_height_error_m` is:

```text
simulated apogee altitude - reference maximum altitude
```

## Outputs

All outputs are written to:

```text
tools/results/sweep/
```

Top-level files:

- `sweep_metrics.csv`
- `sweep_error_summary.png`
- `apogee_altitude_vs_gain_scale.png`
- `all_metrics_vs_gain_scale.png`

Per-metric figures:

```text
tools/results/sweep/metrics/
```

The altitude plots include the reference apogee altitude as a dashed constant line. Error plots include zero in the y-axis limits without drawing a zero reference line.
