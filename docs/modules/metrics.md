# Module: `src/metrics.py`

## Overview

Computes performance indicators for the controlled flight, focusing on tracking accuracy and control effort during the ascent phase.

## Tracking Metrics

For each timestep $i$, the position error $\vec{e}_i$ is defined as the difference between the reference and the real position:

$$\vec{e}_i = \vec{p}_{ref,i} - \vec{p}_{real,i}$$

### 1. Mean Absolute Error (MAE)
Measures the average magnitude of the position error:

$$\text{MAE} = \frac{1}{N} \sum_{i=1}^{N} \|\vec{e}_i\|_2$$

### 2. Root Mean Square Error (RMSE)
Penalizes larger deviations more heavily than MAE:

$$\text{RMSE} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \|\vec{e}_i\|_2^2}$$

## Control Effort Metrics

### 1. Fin Saturation Ratio
The percentage of the control phase where at least one fin was at its deflection limit:

$$R_{sat} = \frac{\text{Samples with } |\delta| \geq \delta_{limit}}{\text{Total Samples}}$$

### 2. Control-Induced Drag
The drag coefficient $C_D$ generated specifically by the fin deflections:

$$C_{D,i} = k \cdot (C_{N,i}^2 + C_{y,i}^2)$$

## Key Outputs

The metrics are exported in `metrics.json` and include:
- **Trajectory Accuracy**: MAE/RMSE for each axis (X, Y, Z).
- **Control Stats**: Maximum and average fin deflections.
- **Flight Summary**: Apogee altitude, maximum Mach number, and control duration.
