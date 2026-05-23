# Running the Simulation

## Prerequisites

- Python 3.12 or newer
- `uv`

Install dependencies from the lockfile:

```bash
uv sync
```

## Nominal Simulation

Run the closed-loop simulation:

```bash
uv run main.py
```

Or use the installed console script:

```bash
uv run rocket-control
```

The run sequence is:

1. `config.load_config()` creates the execution configuration.
2. `initial_data.load_initial_case_data()` loads paths and the rocket TOML.
3. `src.reference.load_reference_trajectory()` loads the configured reference.
4. `src.controllers.build_controller()` creates the mutable controller state.
5. `src.environment_builder.build_environment()` builds the RocketPy environment.
6. `src.rocket_builder.build_rocket()` builds the motor, rocket, passive fins, active `GenericSurface`, and optional parachute.
7. `src.simulation.simulate_controlled_flight()` runs RocketPy `Flight`.
8. `src.metrics.compute_tracking_metrics()` computes tracking and diagnostic metrics.
9. `src.simulation.export_results()` writes CSV, JSON, diagnostics, and plots.

## Gain Scale Sweep

Run:

```bash
uv run py tools/sweep_gain_scale.py
```

The sweep interval is configured at the top of `tools/sweep_gain_scale.py`:

```python
SCALE_MIN = 1.0
SCALE_MAX = 7.0
SCALE_STEP = 0.5
```

For each factor, the tool sets `config.attitude_gain_scale`, rebuilds the controller and rocket, runs the simulation with `terminate_on_apogee = True`, computes apogee-limited metrics, and writes outputs to `tools/results/sweep/`.

## Reference Generation

If the default vertical reference is missing:

```bash
uv run py -c "from src.gen_reference import generate_vertical_reference; generate_vertical_reference('data/trajectory/vertical.csv', max_altitude=1000, duration=20)"
```

Passive trajectory references can also be generated with:

```bash
uv run py tools/trajectory-creator.py
```

## Configuration Summary

`config.py` contains execution parameters, paths, launch site, atmosphere settings, and controller gains.

Attitude gains are calculated from the Ziegler-Nichols baseline gains and the gain-scale factor:

```python
Kp_attitude = Kp_attitude_zn * attitude_gain_scale
Ki_attitude = Ki_attitude_zn * attitude_gain_scale
Kd_attitude = Kd_attitude_zn * attitude_gain_scale
```

The physical control-surface limits are read from the rocket TOML `[control_actuation]` section and copied into the controller state by `src.rocket_builder.build_rocket()`.

## Results

Nominal runs write to `results/<run_id>/`.

Sweep runs write to `tools/results/sweep/`.

See [Input and Output Specifications](io_specs.md) and [Plots and Analysis](plots.md) for output details.
