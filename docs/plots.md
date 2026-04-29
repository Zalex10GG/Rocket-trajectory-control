# Plots and Analysis

The simulation generates 7 analysis plots saved to `results/<run_id>/plots/`. Plots are split between **full-flight** context and **control-phase** detailed analysis.

## Plot Categories

| # | Plot | Scope | Purpose |
|---|------|-------|---------|
| 1 | `trajectory_3d.png` | Full flight | 3D trajectory comparison |
| 2 | `position_per_axis.png` | Control phase | Position tracking |
| 3 | `tracking_errors.png` | Control phase | Error analysis |
| 4 | `fin_actuation.png` | Control phase | Fin deflections |
| 5 | `attitude_euler.png` | Control phase | Euler angles |
| 6 | `body_rates.png` | Control phase | Angular rates |
| 7 | `trajectory_2d_projections.png` | Full flight | Top/side views |

---

## 1. Trajectory 3D (`trajectory_3d.png`)

**Scope**: Full flight (launch to apogee)

**Description**: 3D visualization of the real vs. reference trajectory in local ENU coordinates.

**Axes**:
- X: East (m)
- Y: North (m)
- Z: Up (m)

**Features**:
- Blue solid line: Real trajectory
- Black dashed line: Reference trajectory
- Green circle: Launch point (0,0,0)
- Red X: Apogee (max altitude)

**Purpose**: Verify overall trajectory tracking performance in 3D space.

**Code**: `src/plots.py`, lines 42-58

---

## 2. Position Per-Axis (`position_per_axis.png`)

**Scope**: Control phase only (from first fin deflection to apogee)

**Description**: Per-axis position tracking comparing real vs. reference.

**Subplots** (3 rows, 1 column):
1. **X (East)**: Lateral position tracking
2. **Y (North)**: Lateral position tracking
3. **Z (Up)**: Vertical position tracking

**Axes**:
- X: Time (s)
- Y: Position (m)

**Features**:
- Solid line: Real position
- Dashed red line: Reference position
- Grid for easy reading

**Purpose**: Analyze tracking performance in each axis separately during active control.

**Code**: `src/plots.py`, lines 60-73

---

## 3. Tracking Errors (`tracking_errors.png`)

**Scope**: Control phase

**Description**: Detailed error analysis during active control.

**Subplots** (2 rows, 1 column):

### 3a. Total Tracking Error (Norm)
- **Y-axis**: 3D distance error (m) = `||pos_ref - pos_real||`
- **Purpose**: Overall tracking performance metric

### 3b. Per-Axis Tracking Error
- **Y-axis**: Error per axis (m): `error_x`, `error_y`, `error_z`
- **Purpose**: Identify which axis has the largest deviation

**Interpretation**:
- **Low error**: Good tracking
- **Growing error**: Controller struggling or reference infeasible
- **Oscillations**: Possible instability or excessive gains

**Code**: `src/plots.py`, lines 75-100

---

## 4. Fin Actuation (`fin_actuation.png`)

**Scope**: Control phase

**Description**: History of fin deflections for all 4 fins.

**Axes**:
- X: Time (s)
- Y: Fin deflection (degrees)

**Features**:
- 4 lines (one per fin)
- Positive = trailing edge up (typical convention)
- Saturation limits (if visible) at `±delta_max_rad`

**Interpretation**:
- **All fins similar**: Primarily pitch/yaw control
- **Offset between pairs**: Roll control component
- **Saturation at ±15°**: Max deflection reached (may indicate need for different gains or reference)
- **High-frequency oscillation**: Excessive derivative gain

**Fin Mapping**:
- Fin 1 (0°): Right
- Fin 2 (90°): Top
- Fin 3 (180°): Left
- Fin 4 (270°): Bottom

**Code**: `src/plots.py`, lines 102-114

---

## 5. Attitude Euler Angles (`attitude_euler.png`)

**Scope**: Control phase

**Description**: Rocket attitude represented as Euler angles (roll, pitch, yaw) in degrees.

**Axes**:
- X: Time (s)
- Y: Angle (degrees)

**Features**:
- Roll (φ): Rotation around body X-axis
- Pitch (θ): Rotation around body Y-axis
- Yaw (ψ): Rotation around body Z-axis

**Convention**: ZYX rotation order (yaw first, then pitch, then roll).

**Interpretation**:
- **Roll near 0°**: Good roll stabilization
- **Pitch/Yaw small**: Rocket pointing along desired direction
- **Large angles**: Significant deviation from reference attitude

**Note**: Euler angles are derived from quaternions via `src/utils.py::quaternion_to_euler()`.

**Code**: `src/plots.py`, lines 116-129

---

## 6. Body Angular Rates (`body_rates.png`)

**Scope**: Control phase

**Description**: Angular velocities in the body frame.

**Axes**:
- X: Time (s)
- Y: Angular rate (deg/s)

**Features**:
- ωx (Roll rate): Around body X-axis
- ωy (Pitch rate): Around body Y-axis
- ωz (Yaw rate): Around body Z-axis

**Interpretation**:
- **Near 0**: Stable rotation (good)
- **High values**: Aggressive maneuvering or instability
- **Oscillations**: Possible control chatter or limit cycling

**Purpose**: Verify that the controller is not inducing excessive angular rates that could stress the airframe.

**Code**: `src/plots.py`, lines 131-143

---

## 7. Trajectory 2D Projections (`trajectory_2d_projections.png`)

**Scope**: Full flight

**Description**: Three 2D projections of the trajectory for detailed analysis.

**Subplots**:
1. **Top View (XY)**: East-North plane
2. **Side View (XZ)**: East-Up plane
3. **Profile View (YZ)**: North-Up plane

**Features**:
- Blue line: Real trajectory
- Black dashed line: Reference trajectory
- Equal aspect ratio for correct geometry visualization

**Purpose**: Identify lateral deviations and asymmetry in tracking.

**Code**: `src/plots.py`, lines 145-178

---

## Plot Generation Code

All plots are generated by `src/plots.py::generate_all_plots()`:

```python
def generate_all_plots(flight_history, reference, metrics, config, output_dir=None):
    # Base directory
    run_plots_dir = os.path.join(output_dir, "plots")
    os.makedirs(run_plots_dir, exist_ok=True)
    
    # Identify control window
    start_idx, end_idx = utils.get_control_window_indices(flight_history)
    ctrl_history = flight_history[start_idx:end_idx+1]
    
    # Generate 7 plots...
```

**Control Window**: Plots 3-7 use only the "control phase" data, identified by:
- Start: First timestep with nonzero fin deflection
- End: Apogee (max altitude)

---

## Customizing Plots

To modify plot appearance or add new plots, edit `src/plots.py`.

**Common modifications**:

1. **Change figure size**: Modify `figsize=(width, height)` parameter
2. **Add grid**: `ax.grid(True)` or `ax.grid(False)`
3. **Change colors**: Modify color strings (e.g., `'blue'` → `'red'`)
4. **Adjust font size**: `plt.rcParams.update({'font.size': 12})`

**Adding a new plot**:

```python
# Example: Add acceleration plot
plt.figure(figsize=(10, 6))
# ... plot code ...
plt.savefig(os.path.join(run_plots_dir, "acceleration.png"))
plt.close()
```

---

## Full Flight vs. Control Phase

| Aspect | Full Flight Plots | Control Phase Plots |
|--------|-------------------|---------------------|
| **Plots** | #1, #2 | #3, #4, #5, #6, #7 |
| **Start** | t=0 (launch) | First fin deflection |
| **End** | Apogee | Apogee |
| **Includes** | Motor burn, coast, control | Only active control |
| **Purpose** | Overall trajectory view | Detailed control analysis |

**Why separate?** During motor burn (first ~3.5s), fins are not controlled. Including this phase in control analysis would dilute the metrics and make plots harder to read.
