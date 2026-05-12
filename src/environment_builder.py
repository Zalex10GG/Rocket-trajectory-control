"""
Environment builder for RocketPy simulations.

Supports three atmosphere modes via ``config.atmosphere_type``:

* ``"auto"``   – selects *Reanalysis* (ERA5 via CDS API) for past dates and
  *Forecast* (GFS) for future dates.  ERA5 files are cached in
  ``data/atmosphere/`` so they are downloaded only once.
* ``"Forecast"`` / ``"Reanalysis"`` / ``"standard_atmosphere"`` – used
  directly, skipping auto-detection.

Every remote fetch is wrapped in a try/except that falls back to
``standard_atmosphere`` on failure, so the simulation always runs.
"""

from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path

from rocketpy import Environment

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_ERA5_CACHE_DIR = Path("data/atmosphere")

_ERA5_DATASET = "reanalysis-era5-pressure-levels"
_ERA5_VARIABLES = [
    "geopotential",
    "temperature",
    "u_component_of_wind",
    "v_component_of_wind",
]
_ERA5_PRESSURE_LEVELS = [
    "100", "200", "250", "300", "400",
    "500", "700", "850", "925", "1000",
]
# Bounding box around the launch site (N, W, S, E) — generous margin
_ERA5_AREA = [44, -8, 40, -4]


# ---------------------------------------------------------------------------
# ERA5 download helper
# ---------------------------------------------------------------------------
def _download_era5(launch_date: tuple[int, int, int, int]) -> str:
    """Download an ERA5 pressure-level NetCDF for *launch_date* and return its path.

    The file is cached in ``_ERA5_CACHE_DIR`` so subsequent calls with the
    same date return immediately.

    Parameters
    ----------
    launch_date : tuple
        ``(year, month, day, hour_UTC)`` matching ``config.launch_date``.

    Returns
    -------
    str
        Absolute path to the downloaded ``.nc`` file.

    Raises
    ------
    Exception
        Any error from the CDS API (auth, network, quota, …).
    """
    year, month, day, hour = launch_date
    filename = f"era5_{year}{month:02d}{day:02d}_{hour:02d}00.nc"
    target = _ERA5_CACHE_DIR / filename

    if target.exists():
        print(f"  [CACHE] ERA5 file already exists: {target}")
        return str(target)

    _ERA5_CACHE_DIR.mkdir(parents=True, exist_ok=True)

    import cdsapi  # lazy import — only needed for download

    request = {
        "product_type": ["reanalysis"],
        "variable": _ERA5_VARIABLES,
        "year": [str(year)],
        "month": [f"{month:02d}"],
        "day": [f"{day:02d}"],
        "time": [f"{hour:02d}:00"],
        "pressure_level": _ERA5_PRESSURE_LEVELS,
        "data_format": "netcdf",
        "download_format": "unarchived",
        "area": _ERA5_AREA,
    }

    print(f"  Downloading ERA5 data for {year}-{month:02d}-{day:02d} {hour:02d}:00 UTC ...")
    client = cdsapi.Client()
    client.retrieve(_ERA5_DATASET, request, str(target))
    print(f"  [OK] ERA5 saved to {target}")
    return str(target)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def build_environment(case_data: dict, config) -> Environment:
    """Construct the RocketPy :class:`Environment`.

    Uses ``config.launch_date`` for the simulation date and selects the
    atmospheric model according to ``config.atmosphere_type``:

    * ``"auto"`` — past dates use ERA5 Reanalysis (downloaded automatically
      via the CDS API); future dates use GFS Forecast.
    * ``"Forecast"`` / ``"Reanalysis"`` / ``"standard_atmosphere"`` — used
      as-is.

    On any remote-data failure the function falls back to
    ``standard_atmosphere`` so the simulation can always proceed.
    """
    env = Environment(
        latitude=case_data["latitude"],
        longitude=case_data["longitude"],
        elevation=case_data["elevation_asl_m"],
    )

    # Launch date (year, month, day, hour_UTC)
    env.set_date(config.launch_date)

    atm_type = config.atmosphere_type

    # --- Wind disabled → skip all remote fetching ---------------------------
    if not getattr(config, "use_wind", True):
        env.set_atmospheric_model(type="standard_atmosphere")
        print("  [OK] Atmosphere: standard_atmosphere (use_wind=False)")
        return env

    # --- Auto-detect based on date ------------------------------------------
    if atm_type == "auto":
        year, month, day, hour = config.launch_date
        launch_dt = datetime(year, month, day, hour, tzinfo=timezone.utc)
        now_utc = datetime.now(timezone.utc)

        if launch_dt < now_utc:
            atm_type = "Reanalysis"
            print(f"  [AUTO] Past date detected -> ERA5 Reanalysis")
        else:
            atm_type = "Forecast"
            print(f"  [AUTO] Future date detected -> GFS Forecast")

    # --- Resolve the atmospheric model --------------------------------------
    if atm_type == "Reanalysis":
        try:
            # Use manually-specified file or auto-download via CDS API
            nc_path = getattr(config, "atmosphere_file", None)
            if nc_path and os.path.isfile(nc_path):
                print(f"  Using local ERA5 file: {nc_path}")
            else:
                nc_path = _download_era5(config.launch_date)

            env.set_atmospheric_model(
                type="Reanalysis",
                file=nc_path,
                dictionary="ECMWF",
            )
            print(
                f"  [OK] Atmosphere: Reanalysis loaded "
                f"from '{nc_path}' for {config.launch_date}"
            )
        except Exception as exc:
            print(
                f"  [WARNING] Reanalysis failed ({exc}). "
                "Falling back to standard_atmosphere."
            )
            env.set_atmospheric_model(type="standard_atmosphere")

    elif atm_type == "Forecast":
        try:
            forecast_file = getattr(config, "atmosphere_file", "GFS") or "GFS"
            env.set_atmospheric_model(type="Forecast", file=forecast_file)
            print(
                f"  [OK] Atmosphere: Forecast ({forecast_file}) loaded "
                f"for {config.launch_date}"
            )
        except Exception as exc:
            print(
                f"  [WARNING] Forecast failed ({exc}). "
                "Falling back to standard_atmosphere."
            )
            env.set_atmospheric_model(type="standard_atmosphere")

    else:
        # standard_atmosphere or any other RocketPy type
        env.set_atmospheric_model(type=atm_type)
        print(f"  [OK] Atmosphere: {atm_type}")

    return env
