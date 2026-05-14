# Running the Simulation

## Prerequisites

- **Python**: >= 3.12
- **Package Manager**: [uv](https://github.com/astral-sh/uv)

### Setup
Sync the environment and dependencies:
```bash
uv sync
```

## Execution

### Nominal Simulation
Runs the full closed-loop control simulation:
```bash
uv run main.py
```

### Reference Generation
If the trajectory reference is missing:
```bash
uv run py -c "from src.gen_reference import generate_vertical_reference; generate_vertical_reference('data/trajectory/vertical.csv', max_altitude=1000, duration=20)"
```

## Configuration

All parameters are centralized in `config.py`. Key sections include:

### 1. Control Gains
- `Kp_guidance`, `Kd_guidance`: Outer-loop (trajectory tracking).
- `Kp_attitude`, `Ki_attitude`, `Kd_attitude`: Inner-loop (pitch/yaw attitude).
- `Kp_roll`: Roll rate damping.

### 2. Launch Site
- `latitude`, `longitude`, `elevation_asl_m`: Geodetic location.
- `rail_length_m`: Length of the launch rail.
- `inclination_deg`: 90 for vertical, < 90 for inclined launches.

### 3. Atmosphere
- `atmosphere_type`: "standard", "auto", "Reanalysis", or "Forecast".

## Results

Each run generates a timestamped folder in `results/` (e.g., `results/20260514_203512/`) containing:
- `flight_history.csv`: Time-series of all states and control signals.
- `metrics.json`: Statistical tracking performance (MAE, RMSE).
- `plots/`: Visual analysis of trajectory, attitude, and fins.

## Troubleshooting

- **Module Not Found**: Run `uv sync` to ensure `.venv` is up to date.
- **Atmospheric Data**: Ensure you have an internet connection if using "Reanalysis" or "Forecast" modes for the first time.
- **Reference Feasibility**: If the rocket cannot follow the reference, check if the `delta_max` in the TOML or the aerodynamic coefficients are sufficient for the desired maneuver.
