"""
Environment builder for RocketPy simulations.

Supports automatic atmospheric model selection (standard, reanalysis, forecast)
based on the launch date and configuration.
"""

from __future__ import annotations
import os
from datetime import datetime, timezone
from pathlib import Path
from rocketpy import Environment

_ERA5_CACHE_DIR = Path("data/atmosphere")
_ERA5_DATASET = "reanalysis-era5-pressure-levels"
_ERA5_VARIABLES = ["geopotential", "temperature", "u_component_of_wind", "v_component_of_wind"]
_ERA5_PRESSURE_LEVELS = ["100", "200", "250", "300", "400", "500", "700", "850", "925", "1000"]
_ERA5_AREA = [44, -8, 40, -4] # Bounding box (N, W, S, E)

def _download_era5(launch_date: tuple[int, int, int, int]) -> str:
    """Downloads ERA5 reanalysis data for the specified date."""
    year, month, day, hour = launch_date
    filename = f"era5_{year}{month:02d}{day:02d}_{hour:02d}00.nc"
    target = _ERA5_CACHE_DIR / filename

    if target.exists():
        return str(target)

    _ERA5_CACHE_DIR.mkdir(parents=True, exist_ok=True)
    import cdsapi
    request = {
        "product_type": ["reanalysis"], "variable": _ERA5_VARIABLES,
        "year": [str(year)], "month": [f"{month:02d}"], "day": [f"{day:02d}"], "time": [f"{hour:02d}:00"],
        "pressure_level": _ERA5_PRESSURE_LEVELS, "data_format": "netcdf", "download_format": "unarchived", "area": _ERA5_AREA,
    }
    cdsapi.Client().retrieve(_ERA5_DATASET, request, str(target))
    return str(target)

def build_environment(case_data: dict, config) -> Environment:
    """
    Constructs the RocketPy Environment.
    Single source of truth for location and date is the config object.
    """
    env = Environment(
        latitude=config.latitude,
        longitude=config.longitude,
        elevation=config.elevation_asl_m,
    )
    env.set_date(config.launch_date)

    if not config.use_wind:
        env.set_atmospheric_model(type="standard_atmosphere")
        return env

    atm_type = config.atmosphere_type
    if atm_type == "auto":
        launch_dt = datetime(*config.launch_date, tzinfo=timezone.utc)
        atm_type = "Reanalysis" if launch_dt < datetime.now(timezone.utc) else "Forecast"

    try:
        if atm_type == "Reanalysis":
            nc_path = config.atmosphere_file if config.atmosphere_file and os.path.isfile(config.atmosphere_file) else _download_era5(config.launch_date)
            env.set_atmospheric_model(type="Reanalysis", file=nc_path, dictionary="ECMWF")
        elif atm_type == "Forecast":
            env.set_atmospheric_model(type="Forecast", file=config.atmosphere_file or "GFS")
        else:
            env.set_atmospheric_model(type=atm_type)
    except Exception as exc:
        print(f"  [WARNING] Atmospheric model failed ({exc}). Falling back to standard.")
        env.set_atmospheric_model(type="standard_atmosphere")

    return env
