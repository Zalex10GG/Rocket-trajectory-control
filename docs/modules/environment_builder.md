# Module: `src/environment_builder.py`

## Overview

`src.environment_builder` creates the RocketPy `Environment` from `config.py`.

## `build_environment(case_data, config)`

The environment uses:

- `config.latitude`
- `config.longitude`
- `config.elevation_asl_m`
- `config.launch_date`

If `config.use_wind` is false, the builder sets:

```python
env.set_atmospheric_model(type="standard_atmosphere")
```

If `config.use_wind` is true, it uses `config.atmosphere_type`.

## Atmospheric Modes

- `auto`: selects `Reanalysis` for past launch dates and `Forecast` for future launch dates.
- `Reanalysis`: uses `config.atmosphere_file` if present, otherwise downloads ERA5 data.
- `Forecast`: uses `config.atmosphere_file` or RocketPy's `GFS` option.
- Any other `atmosphere_type`: passed directly to RocketPy.

If atmospheric setup fails, the builder prints a warning and falls back to `standard_atmosphere`.

## ERA5 Cache

Downloaded ERA5 files are stored in:

```text
data/atmosphere/
```

The request uses pressure levels from 100 hPa to 1000 hPa and variables for geopotential, temperature, and horizontal wind components.
