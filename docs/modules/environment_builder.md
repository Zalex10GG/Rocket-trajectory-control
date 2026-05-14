# Module: `src/environment_builder.py`

## Overview

Constructs the RocketPy `Environment` object, which defines the launch site location, date, and atmospheric model.

## Atmospheric Models

Supports three modes controlled by `config.atmosphere_type`:
- **Standard**: Uses the International Standard Atmosphere (ISA).
- **Reanalysis**: Downloads historical ERA5 data for the specified `launch_date` via the CDS API.
- **Forecast**: Downloads GFS forecast data for future dates.

## Key Functions

### `build_environment(case_data, config)`
Creates and configures the environment. It pulls site coordinates directly from `config.py`:
- `latitude`
- `longitude`
- `elevation_asl_m`

### `_download_era5(launch_date)`
Automates the retrieval of NetCDF atmospheric data. It uses a bounding box around the launch site to minimize file size:
- **Area**: $[44^\circ N, 8^\circ W, 40^\circ N, 4^\circ E]$
- **Variables**: Geopotential, temperature, and wind components ($u, v$).

## Atmospheric Properties

The `Environment` object provides the physics engine with real-time data for:
- **Density ($\rho$)**: Used for drag and dynamic pressure calculations.
- **Speed of Sound ($a$)**: Used for Mach number determination.
- **Wind Velocity ($\vec{v}_{wind}$)**: Used to compute the rocket's angle of attack and guidance corrections.

## Implementation Details
- **Date Handling**: Uses `config.launch_date` (tuple: year, month, day, hour) for all temporal calculations.
- **Caching**: Atmospheric files are stored in `data/atmosphere/` to avoid redundant downloads.
- **Fallback**: If remote data fetching fails, the builder automatically falls back to the `standard_atmosphere` to ensure simulation continuity.
