# Module: `src/environment_builder.py`

## Overview

Constructs the **RocketPy Environment** object with launch site parameters and atmospheric model.

**Note**: Launch site parameters are passed via `case_data` (which gets them from `config.py`), not hardcoded.

## Key Functions

### `build_environment(case_data, config)`

**Purpose**: Creates and configures a RocketPy `Environment` object for the simulation.

**Signature**:
```python
def build_environment(case_data: dict, config: object) -> RocketPy.Environment:
```

**Implementation**:
```python
def build_environment(case_data, config):
    env = Environment(
        latitude=case_data["latitude"],    # From config.py via initial_data.py
        longitude=case_data["longitude"],  # From config.py
        elevation=case_data["elevation_asl_m"]  # From config.py
    )
    
    # Set fixed date for reproducibility
    env.set_date((2026, 4, 28, 12))
    
    # ISA Atmosphere (default in RocketPy if no ensemble/forecast is set)
    env.set_atmospheric_model(type='standard_atmosphere')
    
    return env
```

**Returns**: Configured `RocketPy.Environment` object.

**Gravity access**: `env.gravity(elevation)` is used in `simulation.py` to get gravity magnitude for the controller.

---

## Parameters

### Launch Site (from `initial_data.py`)

| Parameter | Source | Description | Units |
|-----------|--------|-------------|-------|
| `latitude` | `case_data["latitude"]` | Launch site latitude | degrees |
| `longitude` | `case_data["longitude"]` | Launch site longitude | degrees |
| `elevation` | `case_data["elevation_asl_m"]` | Launch site elevation | meters ASL |

**Default values** (Leon 2 launch site):
```python
latitude = 42.3402247448   # ~42.34°N
longitude = -6.2713407985   # ~6.27°W
elevation_asl_m = 1000.0    # 1000m ASL
```

### Atmospheric Model

**Type**: `standard_atmosphere` (ISA - International Standard Atmosphere)

**Characteristics**:
- Temperature lapse rate: 6.5 K/km (troposphere)
- Sea level temperature: 288.15 K (15°C)
- Sea level pressure: 101325 Pa
- Sea level density: 1.225 kg/m³
- No wind modeling

**Fixed Date**: `(2026, 4, 28, 12)` (April 28, 2026, 12:00 UTC)

Setting a fixed date ensures **reproducible results** (RocketPy's environment can vary with date due to atmospheric conditions).

---

## Dependencies

- `rocketpy`: `Environment` class
- `numpy`: (implicit, used by RocketPy)

---

## RocketPy Environment Features

The `Environment` object provides:

1. **Atmospheric Properties**:
   - Temperature vs. altitude
   - Pressure vs. altitude
   - Density vs. altitude
   - Speed of sound vs. altitude
   - Dynamic viscosity vs. altitude

2. **Wind Models** (not used currently):
   - Constant wind
   - Custom wind profiles
   - Ensemble forecasts (GFS, etc.)

3. **Geodetic Calculations**:
   - Geodetic to ECEF conversions
   - Local horizon (ENU) transformations

---

## Usage in Simulation

```python
import initial_data as init
import src.environment_builder as env_builder
import config as cfg

# Load data and config
config = cfg.load_config()
case_data = init.load_initial_case_data(config)

# Build environment
environment = env_builder.build_environment(case_data, config)

# Use in flight simulation
from rocketpy import Flight
flight = Flight(
    rocket=rocket,
    environment=environment,
    ...
)
```

---

## Caveats and Notes

1. **No Wind Modeling**: The current setup uses standard ISA atmosphere with **no wind**. Real launches experience wind gusts and shear that significantly affect trajectory.

2. **Fixed Date**: While good for reproducibility, the fixed date may not represent actual launch conditions. For realism, consider:
   - Using actual launch date weather
   - Adding wind profiles
   - Using ensemble forecasts for uncertainty quantification

3. **Atmospheric Model**: `standard_atmosphere` is a simple model. RocketPy supports:
   - `custom_atmosphere`: Load custom atmospheric data
   - Ensemble forecasts from weather models

4. **Launch Site**: The launch site coordinates are hardcoded in `initial_data.py`. For different launch sites, modify `data/rockets/leon_2.toml` or `initial_data.py`.

5. **Elevation**: The elevation in `case_data` should match the launch site ASL. This affects:
   - Initial air density (affects drag)
   - Initial temperature (affects motor performance)
   - Atmospheric pressure (affects motor nozzle expansion)

---

## Future Enhancements

1. **Wind Modeling**:
   ```python
   env.set_atmospheric_model(
       type='custom_atmosphere',
       wind_u=[(0, 0), (1000, 10), (5000, 20)],  # Altitude vs. wind speed (m/s)
       wind_v=[(0, 0), (1000, 5), (5000, 10)]
   )
   ```

2. **Date from Config**:
   ```python
   env.set_date((config.launch_year, config.launch_month, config.launch_day, 12))
   ```

3. **Atmospheric Sensitivity**: Run Monte Carlo with varying atmospheric conditions.

---

## Example Usage

```python
from src.environment_builder import build_environment
import initial_data as init
import config as cfg

# Load launch site data
config = cfg.load_config()
case_data = init.load_initial_case_data(config)
# case_data contains: latitude, longitude, elevation_asl_m

# Build environment
env = build_environment(case_data, config)

# Inspect atmospheric properties
altitude = 1000  # m ASL
print(f"Temperature at {altitude}m: {env.temperature(altitude)} K")
print(f"Pressure at {altitude}m: {env.pressure(altitude)} Pa")
print(f"Density at {altitude}m: {env.density(altitude)} kg/m³")
```
