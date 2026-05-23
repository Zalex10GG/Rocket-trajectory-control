# `tools/trajectory-creator.py`

## Purpose

Generates a passive, uncontrolled flight simulation and exports it as a CSV reference trajectory. This provides a baseline trajectory (e.g., a vertical launch or a ballistic arc) for the controlled simulation to track.

## Command

Run from the repository root:

```powershell
uv run py tools/trajectory-creator.py
```

## Workflow

1.  **Passive Simulation**: Loads the rocket and motor data to run a standard RocketPy flight without control logic.
2.  **Coordinate Transformation**: Converts the geodetic trajectory to a local **East-North-Up (ENU)** frame centered at the launch pad:
    $$\vec{p}_{ENU} = \vec{p}_{abs} - \vec{p}_{launch}$$
3.  **Reference Export**: Saves the time-series data (Position and Velocity) in a format compatible with the main simulation's reference loader.

## Outputs

- **Trajectory CSV**: written to the output path configured in `tools/trajectory-creator.py`. Contains columns for time, $x, y, z$ positions, and $v_x, v_y, v_z$ velocities in local ENU.
- **Diagnostic Plots**: Visualizations of the thrust curve, static margin, and 3D flight path are saved in `data/trajectory/plots/`.
