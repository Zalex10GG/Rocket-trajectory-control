"""Sweep attitude gain scale and export apogee-limited tracking results.

The script runs one closed-loop simulation per gain factor up to apogee,
computes 3D mean error, max 3D error, apogee 3D/lateral errors, and
maximum-height error, then writes one CSV plus plots under
``tools/results/sweep``.

Usage:
    uv run py tools/sweep_gain_scale.py
"""

# ===========================================================================
# SWEEP INTERVAL CONFIGURATION (Adjust limits and step here)
# ===========================================================================
SCALE_MIN = 1.0
SCALE_MAX = 7.0
SCALE_STEP = 0.5
# ===========================================================================

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Add project root to path if needed
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config as cfg
import initial_data as init
import src.controllers as controllers
import src.environment_builder as env_builder
import src.reference as reference_mod
import src.rocket_builder as rocket_builder
import src.simulation as sim
from src.plots import save_figure


OUTPUT_DIR = os.path.join("tools", "results", "sweep")


def build_gain_scales(scale_min, scale_max, scale_step):
    """Return an inclusive sweep grid with basic interval validation."""
    if scale_step <= 0.0:
        raise ValueError("SCALE_STEP must be positive.")
    if scale_max < scale_min:
        raise ValueError("SCALE_MAX must be greater than or equal to SCALE_MIN.")

    count = int(np.floor((scale_max - scale_min) / scale_step + 1e-9)) + 1
    scales = scale_min + scale_step * np.arange(count)
    if scales[-1] < scale_max - 1e-9:
        scales = np.append(scales, scale_max)
    return np.round(scales, decimals=12)


def compute_apogee_limited_errors(flight_history, reference):
    """Compute position errors over the available ascent history up to apogee."""
    if not flight_history:
        raise ValueError("No flight history data available.")

    positions = np.array([sample["position_enu_m"] for sample in flight_history])
    times = np.array([sample["time_s"] for sample in flight_history])
    apogee_idx = int(np.argmax(positions[:, 2]))

    times_to_apogee = times[: apogee_idx + 1]
    positions_to_apogee = positions[: apogee_idx + 1]
    ref_positions_to_apogee = np.array(
        [
            reference_mod.sample_reference(reference, time_s)["position_enu_m"]
            for time_s in times_to_apogee
        ]
    )

    error_vectors = ref_positions_to_apogee - positions_to_apogee
    error_norms = np.linalg.norm(error_vectors, axis=1)
    lateral_error_norms = np.linalg.norm(error_vectors[:, :2], axis=1)
    max_fin_deflection_deg = float(
        np.max(np.abs(np.array([sample["deltas"] for sample in flight_history])))
        * 180.0
        / np.pi
    )

    apogee_time = float(times[apogee_idx])
    apogee_position = positions[apogee_idx]
    reference_at_apogee = reference_mod.sample_reference(reference, apogee_time)
    reference_apogee_position = reference_at_apogee["position_enu_m"]
    apogee_error_vector = reference_apogee_position - apogee_position

    reference_peak_altitude = float(reference.get("peak_z_enu", np.nan))
    if not np.isfinite(reference_peak_altitude):
        ref_z = [
            reference_mod.sample_reference(reference, time_s)["position_enu_m"][2]
            for time_s in reference["time_s"]
        ]
        reference_peak_altitude = float(np.max(ref_z))

    return {
        "mean_3d_error_m": float(np.mean(error_norms)),
        "max_3d_error_m": float(np.max(error_norms)),
        "max_lateral_error_m": float(np.max(lateral_error_norms)),
        "apogee_3d_error_m": float(np.linalg.norm(apogee_error_vector)),
        "apogee_lateral_error_m": float(np.linalg.norm(apogee_error_vector[:2])),
        "apogee_height_error_m": float(apogee_position[2] - reference_peak_altitude),
        "apogee_altitude_real_m": float(apogee_position[2]),
        "reference_max_altitude_m": reference_peak_altitude,
        "reference_altitude_at_apogee_time_m": float(reference_apogee_position[2]),
        "apogee_time_s": apogee_time,
        "max_fin_deflection_deg": max_fin_deflection_deg,
    }


def save_sweep_plots(df, output_dir):
    """Save overview plots and one plot per numeric metric versus gain scale."""
    x_col = "gain_scale"

    def metric_label(column):
        """Return a plot-friendly axis label for a metric column."""
        labels = {
            "gain_scale": "Gain scale",
            "kp_attitude": "Kp attitude",
            "ki_attitude": "Ki attitude",
            "kd_attitude": "Kd attitude",
            "mean_3d_error_m": "Mean 3D error (m)",
            "max_3d_error_m": "Max 3D error (m)",
            "max_lateral_error_m": "Max lateral error (m)",
            "apogee_3d_error_m": "Apogee 3D error (m)",
            "apogee_lateral_error_m": "Apogee lateral error (m)",
            "apogee_height_error_m": "Apogee height error (m)",
            "apogee_altitude_real_m": "Simulated apogee altitude (m)",
            "reference_max_altitude_m": "Reference apogee altitude (m)",
            "reference_altitude_at_apogee_time_m": "Reference altitude at simulated apogee time (m)",
            "apogee_time_s": "Apogee time (s)",
            "max_fin_deflection_deg": "Max fin deflection (deg)",
        }
        return labels.get(column, column.replace("_", " ").capitalize())

    def metric_title(column):
        """Return a plot-friendly title without units."""
        title = metric_label(column)
        for suffix in [" (m)", " (s)", " (deg)"]:
            title = title.replace(suffix, "")
        return title

    def include_zero_on_axis(axis, values):
        """Expand y limits only when needed so zero remains visible."""
        finite_values = np.asarray(values, dtype=float)
        finite_values = finite_values[np.isfinite(finite_values)]
        if finite_values.size == 0:
            return
        y_min = min(float(np.min(finite_values)), 0.0)
        y_max = max(float(np.max(finite_values)), 0.0)
        if np.isclose(y_min, y_max):
            padding = max(abs(y_min) * 0.05, 1.0)
        else:
            padding = 0.05 * (y_max - y_min)
        axis.set_ylim(y_min - padding, y_max + padding)

    def add_reference_apogee_line(axis, df):
        """Add the constant reference apogee altitude line to an altitude plot."""
        axis.axhline(
            df["reference_max_altitude_m"].iloc[0],
            color="black",
            linestyle="--",
            linewidth=1.8,
            label="Reference apogee altitude",
        )
        axis.legend()

    overview_metrics = [
        ("mean_3d_error_m", "Mean 3D Error (m)", "Mean 3D Tracking Error"),
        ("max_3d_error_m", "Max 3D Error (m)", "Max 3D Tracking Error"),
        ("apogee_3d_error_m", "3D Error at Apogee (m)", "Apogee 3D Position Error"),
        (
            "apogee_height_error_m",
            "Height Error at Apogee (m)",
            "Apogee Height Error (Real Peak - Reference Peak)",
        ),
        (
            "apogee_lateral_error_m",
            "Lateral Error at Apogee (m)",
            "Apogee Lateral Error",
        ),
        ("max_lateral_error_m", "Max Lateral Error (m)", "Max Lateral Error"),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    for axis, (column, ylabel, title) in zip(axes.ravel(), overview_metrics):
        axis.plot(df[x_col], df[column], "o-", linewidth=2)
        if "error" in column:
            include_zero_on_axis(axis, df[column])
        axis.set_xlabel("Attitude gain scale factor")
        axis.set_ylabel(ylabel)
        axis.set_title(f"{title} vs Gain Scale")
        axis.grid(True)

    for axis in axes.ravel()[len(overview_metrics) :]:
        axis.axis("off")

    fig.tight_layout()
    overview_path = os.path.join(output_dir, "sweep_error_summary.png")
    save_figure(fig, overview_path, dpi=150)
    plt.close(fig)

    altitude_path = os.path.join(output_dir, "apogee_altitude_vs_gain_scale.png")
    fig, axis = plt.subplots(figsize=(9, 5))
    axis.plot(
        df[x_col],
        df["apogee_altitude_real_m"],
        "o-",
        linewidth=2,
        label="Simulated apogee altitude",
    )
    axis.axhline(
        df["reference_max_altitude_m"].iloc[0],
        color="black",
        linestyle="--",
        linewidth=1.8,
        label="Reference apogee altitude",
    )
    axis.set_xlabel("Attitude gain scale factor")
    axis.set_ylabel("Altitude (m)")
    axis.set_title("Apogee altitude vs gain scale")
    axis.grid(True)
    axis.legend()
    fig.tight_layout()
    save_figure(fig, altitude_path, dpi=150)
    plt.close(fig)

    excluded_generic_columns = {
        x_col,
        "reference_max_altitude_m",
        "reference_altitude_at_apogee_time_m",
    }
    numeric_columns = [
        column
        for column in df.select_dtypes(include=[np.number]).columns
        if column not in excluded_generic_columns
    ]
    n_cols = 3
    n_rows = int(np.ceil(len(numeric_columns) / n_cols))
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, max(4, 3.2 * n_rows)))
    axes_flat = np.atleast_1d(axes).ravel()

    for axis, column in zip(axes_flat, numeric_columns):
        axis.plot(df[x_col], df[column], "o-", linewidth=1.8)
        if column == "apogee_altitude_real_m":
            add_reference_apogee_line(axis, df)
        if "error" in column:
            include_zero_on_axis(axis, df[column])
        axis.set_xlabel("Gain scale")
        axis.set_ylabel(metric_label(column))
        axis.set_title(metric_title(column))
        axis.grid(True)

    for axis in axes_flat[len(numeric_columns) :]:
        axis.axis("off")

    fig.tight_layout()
    all_metrics_path = os.path.join(output_dir, "all_metrics_vs_gain_scale.png")
    save_figure(fig, all_metrics_path, dpi=150)
    plt.close(fig)

    per_metric_dir = os.path.join(output_dir, "metrics")
    os.makedirs(per_metric_dir, exist_ok=True)
    for filename in os.listdir(per_metric_dir):
        if filename.endswith(".png"):
            os.remove(os.path.join(per_metric_dir, filename))

    per_metric_svg_dir = os.path.join(per_metric_dir, "svg")
    if os.path.isdir(per_metric_svg_dir):
        for filename in os.listdir(per_metric_svg_dir):
            if filename.endswith(".svg"):
                os.remove(os.path.join(per_metric_svg_dir, filename))

    for column in numeric_columns:
        fig, axis = plt.subplots(figsize=(8, 5))
        axis.plot(df[x_col], df[column], "o-", linewidth=2)
        if column == "apogee_altitude_real_m":
            add_reference_apogee_line(axis, df)
        if "error" in column:
            include_zero_on_axis(axis, df[column])
        axis.set_xlabel("Attitude gain scale factor")
        axis.set_ylabel(metric_label(column))
        axis.set_title(f"{metric_title(column)} vs gain scale")
        axis.grid(True)
        fig.tight_layout()
        save_figure(fig, os.path.join(per_metric_dir, f"{column}.png"), dpi=150)
        plt.close(fig)

    return overview_path, altitude_path, all_metrics_path, per_metric_dir

def run_sweep():
    print("=====================================================================")
    print("         ROCKET PITCH/YAW ATTITUDE GAIN SCALE SWEEP TOOL             ")
    print("=====================================================================")
    print(f"Interval: [{SCALE_MIN:.2f}, {SCALE_MAX:.2f}] | Step: {SCALE_STEP:.2f}")
    print("Simulation range: UP TO APOGEE ONLY")
    print("=====================================================================")
    
    # Create output directory
    output_dir = OUTPUT_DIR
    os.makedirs(output_dir, exist_ok=True)
    
    # 1. Load nominal configuration and data
    config = cfg.load_config()
    config.save_results = False          # Do not save standard CSVs/plots during sweep
    config.show_plots = False
    config.terminate_on_apogee = True    # Stop integration exactly at apogee
    
    case_data = init.load_initial_case_data(config)
    
    print(f"Loading reference trajectory: {config.reference_path}")
    reference = reference_mod.load_reference_trajectory(config.reference_path)
    
    print("Building environment...")
    environment = env_builder.build_environment(case_data, config)
    
    # Define the range of gain scales to sweep
    scales = build_gain_scales(SCALE_MIN, SCALE_MAX, SCALE_STEP)
    results = []
    
    print("\nStarting sweep...")
    print(f"{'Gain Scale':^12} | {'Kp_att':^10} | {'Mean 3D (m)':^12} | {'Max 3D (m)':^12} | {'Apogee 3D (m)':^14} | {'Peak Alt Err (m)':^16} | {'Apogee Time (s)':^16}")
    print("-" * 105)
    
    for scale in scales:
        # Update scale factor
        config.attitude_gain_scale = float(scale)
        
        # Re-initialize controller state with the new config properties
        controller = controllers.build_controller(config)
        
        # Re-build rocket model to register updated controller and surface mixer
        rocket, components = rocket_builder.build_rocket(case_data, config, controller)
        
        try:
            # Run simulation programmatically (integrates up to apogee due to terminate_on_apogee=True)
            flight_history = sim.simulate_controlled_flight(
                rocket=rocket,
                environment=environment,
                reference=reference,
                controller=controller,
                config=config,
            )
            
            errors = compute_apogee_limited_errors(flight_history, reference)
            
            # Retrieve active proportional gain for logging
            kp = config.Kp_attitude
            ki = config.Ki_attitude
            kd = config.Kd_attitude
            
            print(f"{scale:^12.2f} | {kp:^10.6f} | {errors['mean_3d_error_m']:^12.2f} | {errors['max_3d_error_m']:^12.2f} | {errors['apogee_3d_error_m']:^14.2f} | {errors['apogee_height_error_m']:^16.2f} | {errors['apogee_time_s']:^16.2f}")
            
            results.append({
                "gain_scale": float(scale),
                "kp_attitude": kp,
                "ki_attitude": ki,
                "kd_attitude": kd,
                **errors,
            })
            
        except Exception as e:
            print(f"{scale:^12.2f} | FAILED due to: {str(e)}")
            
    print("-" * 105)
    
    # 2. Analyze and Export results
    if not results:
        print("No valid simulation runs completed.")
        return
        
    df = pd.DataFrame(results)
    csv_path = os.path.join(output_dir, "sweep_metrics.csv")
    df.to_csv(csv_path, index=False)
    print(f"\nCSV results exported to {csv_path}")
    
    overview_path, altitude_path, all_metrics_path, per_metric_dir = save_sweep_plots(df, output_dir)
    print(f"Sweep error summary exported to {overview_path}")
    print(f"Apogee altitude figure exported to {altitude_path}")
    print(f"All-metrics figure exported to {all_metrics_path}")
    print(f"Per-metric plots exported to {per_metric_dir}")
    
    # 3. Print Best Results Recommendations
    best_by_mae = df.loc[df["mean_3d_error_m"].idxmin()]
    best_by_apogee_3d = df.loc[df["apogee_3d_error_m"].idxmin()]
    best_by_apogee_alt = df.loc[df["apogee_height_error_m"].abs().idxmin()]
    
    print("\n========================= SWEEP SUMMARY =========================")
    print(f"Optimal factor for MINIMUM MEAN ERROR (3D MAE):")
    print(f"  -> Scale Factor:   {best_by_mae['gain_scale']:.2f}")
    print(f"  -> 3D MAE:         {best_by_mae['mean_3d_error_m']:.2f} m")
    print(f"  -> Max 3D Error:   {best_by_mae['max_3d_error_m']:.2f} m")
    print(f"  -> Active Gains:   Kp={best_by_mae['kp_attitude']:.6f}, Ki={best_by_mae['ki_attitude']:.6f}, Kd={best_by_mae['kd_attitude']:.6f}")
    print()
    print(f"Optimal factor for MINIMUM APOGEE 3D POSITION ERROR:")
    print(f"  -> Scale Factor:   {best_by_apogee_3d['gain_scale']:.2f}")
    print(f"  -> Apogee 3D Err:  {best_by_apogee_3d['apogee_3d_error_m']:.2f} m")
    print(f"  -> Peak Alt Err:   {best_by_apogee_3d['apogee_height_error_m']:.2f} m")
    print()
    print(f"Optimal factor for MINIMUM APOGEE HEIGHT ERROR:")
    print(f"  -> Scale Factor:   {best_by_apogee_alt['gain_scale']:.2f}")
    print(f"  -> Peak Alt Err:   {best_by_apogee_alt['apogee_height_error_m']:.2f} m")
    print(f"  -> Apogee 3D Err:  {best_by_apogee_alt['apogee_3d_error_m']:.2f} m")
    print("=================================================================")
    print("\nTip: Set 'attitude_gain_scale = {:.2f}' in config.py for minimum trajectory error.".format(best_by_mae['gain_scale']))

if __name__ == "__main__":
    run_sweep()
