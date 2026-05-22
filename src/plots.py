"""
Plot generation for flight simulation results.

Two output subdirectories are produced per run:
- plots/simulation/   : full-flight plots (trajectory, rocket, motor)
- plots/control/       : control-phase-only plots (tracking, fin actuation, etc.)
"""

import os
import matplotlib.pyplot as plt
import numpy as np
import src.reference as ref_mod
import src.utils as utils


# ---------------------------------------------------------------------------
# Helper: save RocketPy plots without interactive windows
# ---------------------------------------------------------------------------

def save_rocketpy_plot(plot_func, path):
    """
    Executes a RocketPy plotting function, suppresses interactive windows,
    and saves the figure to ``path``.

    Parameters
    ----------
    plot_func : callable
        Zero-argument function that triggers RocketPy plot generation.
    path : str
        Destination file path (PNG recommended).
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    plt.close("all")
    original_show = plt.show
    plt.show = lambda *args, **kwargs: None
    try:
        plot_func()
        if plt.get_fignums():
            plt.savefig(path, bbox_inches="tight")
    except Exception as e:
        print(f"WARNING: Failed to save RocketPy plot {path}: {e}")
    finally:
        plt.show = original_show
        plt.close("all")


# ---------------------------------------------------------------------------
# Core generators
# ---------------------------------------------------------------------------

def generate_all_plots(flight_history, reference, metrics, config, output_dir=None, controller_state=None):
    """
    Generates and saves all performance plots split into two subdirectories.

    - ``plots/simulation/`` contains full-flight trajectory plots.
    - ``plots/control/`` contains control-phase-only plots.

    Parameters
    ----------
    flight_history : list[dict]
        Flight state records as produced by ``simulate_controlled_flight``.
    reference : callable
        Reference trajectory sampler (see ``src.reference``).
    metrics : dict
        Metrics dictionary (unused here, kept for API compatibility).
    config : Config
        Configuration object.
    output_dir : str, optional
        Parent output directory. Defaults to ``config.results_dir``.
    """
    if not flight_history:
        print("No flight history to plot.")
        return

    if output_dir is None:
        output_dir = config.results_dir

    sim_dir = os.path.join(output_dir, "simulation")
    ctrl_dir = os.path.join(output_dir, "control")
    os.makedirs(sim_dir, exist_ok=True)
    os.makedirs(ctrl_dir, exist_ok=True)

    times = np.array([s["time_s"] for s in flight_history])

    # Full-flight position arrays (real)
    pos_real_local = np.array([s["position_enu_m"] for s in flight_history])
    
    # Sample reference exactly at the real flight times (needed for control plots)
    ref_states_matched = [ref_mod.sample_reference(reference, t) for t in times]
    pos_ref_local_matched = np.array([r["position_enu_m"] for r in ref_states_matched])
    
    # Sample reference over its ENTIRE timeline (for the full trajectory plots)
    ref_times_full = reference['time_s']
    ref_states_full = [ref_mod.sample_reference(reference, t) for t in ref_times_full]
    pos_ref_local_full = np.array([r["position_enu_m"] for r in ref_states_full])

    # Active-control window (Task 5: use active window, not ascent-to-apogee)
    start_idx, end_idx = utils.get_active_control_window_indices(
        flight_history, controller_state=controller_state
    )
    ctrl_history = flight_history[start_idx : end_idx + 1]
    ctrl_times = times[start_idx : end_idx + 1]
    pos_real_ctrl = pos_real_local[start_idx : end_idx + 1]
    pos_ref_ctrl = pos_ref_local_matched[start_idx : end_idx + 1]

    # Extract velocity arrays for plotting
    vel_real_local = np.array([s["velocity_enu_m_s"] for s in flight_history])
    vel_ref_local_matched = np.array([r["velocity_enu_m_s"] for r in ref_states_matched])
    vel_real_ctrl = vel_real_local[start_idx : end_idx + 1]
    vel_ref_ctrl = vel_ref_local_matched[start_idx : end_idx + 1]

    # ------------------------------------------------------------------
    # plots/simulation/  (full flight)
    # ------------------------------------------------------------------
    show_plots = bool(getattr(config, "show_plots", False))

    _plot_trajectory_3d(pos_real_local, pos_ref_local_full, sim_dir, keep_open=show_plots)
    _plot_trajectory_2d(pos_real_local, pos_ref_local_full, sim_dir, keep_open=show_plots)
    _plot_velocity_per_axis(times, vel_real_local, vel_ref_local_matched, sim_dir, label_prefix="Full Flight: ", keep_open=show_plots)
    
    # Drag Coefficient Cd vs Mach plot (capped at max flight Mach)
    machs = np.array([s.get("mach", 0.0) for s in flight_history])
    max_mach = np.max(machs) if len(machs) > 0 else 0.0
    _plot_cd_vs_mach(max_mach, config, sim_dir, keep_open=show_plots)

    # ------------------------------------------------------------------
    # plots/control/  (control phase only)
    # ------------------------------------------------------------------
    _plot_position_per_axis(ctrl_times, pos_real_ctrl, pos_ref_ctrl, ctrl_dir, keep_open=show_plots)
    _plot_tracking_errors(ctrl_times, pos_real_ctrl, pos_ref_ctrl, ctrl_dir, keep_open=show_plots)
    _plot_fin_actuation(ctrl_times, ctrl_history, ctrl_dir, config=config, controller_state=controller_state, keep_open=show_plots)
    _plot_attitude_euler(ctrl_times, ctrl_history, ctrl_dir, keep_open=show_plots)
    _plot_body_rates(ctrl_times, ctrl_history, ctrl_dir, keep_open=show_plots)
    _plot_velocity_per_axis(ctrl_times, vel_real_ctrl, vel_ref_ctrl, ctrl_dir, label_prefix="Control Phase: ", keep_open=show_plots)
    _plot_trajectory_3d(pos_real_ctrl, pos_ref_ctrl, ctrl_dir, label_prefix="Control Phase: ", keep_open=show_plots)
    _plot_trajectory_2d(pos_real_ctrl, pos_ref_ctrl, ctrl_dir, label_prefix="Control Phase: ", keep_open=show_plots)
    _plot_gain_evolution(ctrl_times, ctrl_history, ctrl_dir, config=config, keep_open=show_plots)

    print(f"Plots generated in {output_dir}")
    if show_plots:
        plt.show()


def generate_rocket_creation_plots(rocket, components, output_dir):
    """
    Saves RocketPy-provided static plots (rocket diagram, static margin,
    motor thrust curve) into ``plots/simulation/``.

    Parameters
    ----------
    rocket : rocketpy.Rocket
        The built Rocket object.
    components : dict
        Component dictionary as returned by ``build_rocket(..., return_components=True)``.
    output_dir : str
        Parent output directory (run folder).
    """
    sim_dir = os.path.join(output_dir, "simulation")
    os.makedirs(sim_dir, exist_ok=True)

    motor = components.get("motor")
    if motor is not None:
        save_rocketpy_plot(motor.plots.thrust, os.path.join(sim_dir, "motor_thrust.png"))

    save_rocketpy_plot(rocket.plots.static_margin, os.path.join(sim_dir, "static_margin.png"))
    save_rocketpy_plot(rocket.plots.draw, os.path.join(sim_dir, "rocket.png"))


# ---------------------------------------------------------------------------
# Internal plotting helpers (single-purpose, save to given directory)
# ---------------------------------------------------------------------------

def _plot_trajectory_3d(pos_real, pos_ref, out_dir, label_prefix="", keep_open=False):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        pos_real[:, 0], pos_real[:, 1], pos_real[:, 2],
        label="Real Trajectory (Local ENU)", color="blue", linewidth=2
    )
    ax.plot(
        pos_ref[:, 0], pos_ref[:, 1], pos_ref[:, 2],
        "k--", label="Reference (Local ENU)", alpha=0.7
    )
    start_label = "Launch" if not label_prefix else "Control Start"
    ax.scatter(pos_real[0, 0], pos_real[0, 1], pos_real[0, 2],
               color="green", marker="o", s=50, label=start_label)
    idx_apogee = np.argmax(pos_real[:, 2])
    ax.scatter(pos_real[idx_apogee, 0], pos_real[idx_apogee, 1], pos_real[idx_apogee, 2],
               color="red", marker="x", s=50, label="Apogee")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(f"{label_prefix}3D Trajectory Tracking (Local Origin)")
    ax.legend()
    plt.savefig(os.path.join(out_dir, "trajectory_3d.png"))
    if not keep_open:
        plt.close(fig)


def _plot_trajectory_2d(pos_real, pos_ref, out_dir, label_prefix="", keep_open=False):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    axes[0].plot(pos_real[:, 0], pos_real[:, 1], label="Real", color="blue")
    axes[0].plot(pos_ref[:, 0], pos_ref[:, 1], "k--", label="Ref", alpha=0.5)
    axes[0].set_xlabel("East (m)")
    axes[0].set_ylabel("North (m)")
    axes[0].set_title(f"{label_prefix}Top View (XY)")
    axes[0].grid(True)
    axes[0].legend()
    axes[0].axis("equal")

    axes[1].plot(pos_real[:, 0], pos_real[:, 2], label="Real", color="blue")
    axes[1].plot(pos_ref[:, 0], pos_ref[:, 2], "k--", label="Ref", alpha=0.5)
    axes[1].set_xlabel("East (m)")
    axes[1].set_ylabel("Up (m)")
    axes[1].set_title(f"{label_prefix}Side View (XZ)")
    axes[1].grid(True)
    axes[1].axis("equal")

    axes[2].plot(pos_real[:, 1], pos_real[:, 2], label="Real", color="blue")
    axes[2].plot(pos_ref[:, 1], pos_ref[:, 2], "k--", label="Ref", alpha=0.5)
    axes[2].set_xlabel("North (m)")
    axes[2].set_ylabel("Up (m)")
    axes[2].set_title(f"{label_prefix}Profile View (YZ)")
    axes[2].grid(True)
    axes[2].axis("equal")

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "trajectory_2d_projections.png"))
    if not keep_open:
        plt.close(fig)


def _plot_position_per_axis(times, pos_real, pos_ref, out_dir, keep_open=False):
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ["X (East)", "Y (North)", "Z (Up)"]
    for i in range(3):
        axes[i].plot(times, pos_real[:, i], label=f"Real {labels[i]}")
        axes[i].plot(times, pos_ref[:, i], "r--", label=f"Ref {labels[i]}")
        axes[i].set_ylabel("Position Local (m)")
        if i == 0:
            axes[i].legend(loc="lower left")
        else:
            axes[i].legend(loc="upper left")
        axes[i].grid(True)
    axes[2].set_xlabel("Time (s)")
    fig.suptitle("Control Phase: Per-Axis Position Tracking")
    plt.tight_layout(rect=(0, 0.03, 1, 0.95))
    plt.savefig(os.path.join(out_dir, "position_per_axis.png"))
    if not keep_open:
        plt.close(fig)


def _plot_tracking_errors(times, pos_real, pos_ref, out_dir, keep_open=False):
    errors = pos_ref - pos_real
    error_norm = np.linalg.norm(errors, axis=1)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    ax1.plot(times, error_norm, color="purple", label="3D Distance Error")
    ax1.set_ylabel("Error (m)")
    ax1.set_title("Control Phase: Total Tracking Error (Norm)")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(times, errors[:, 0], label="Error X")
    ax2.plot(times, errors[:, 1], label="Error Y")
    ax2.plot(times, errors[:, 2], label="Error Z")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Error (m)")
    ax2.set_title("Control Phase: Per-Axis Tracking Error")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "tracking_errors.png"))
    if not keep_open:
        plt.close(fig)


def _plot_fin_actuation(times, ctrl_history, out_dir, config, controller_state=None, keep_open=False):
    fig = plt.figure(figsize=(10, 6))
    deltas = np.array([s["deltas"] for s in ctrl_history]) * 180 / np.pi
    
    # Calculate dynamic maximum deflection limit lines
    limits_deg = []
    delta_max_rad = controller_state.get("delta_max_rad", 0.34906585) if (controller_state and isinstance(controller_state, dict)) else 0.34906585
    for s in ctrl_history:
        q_dynamic = s.get("q_dynamic", 0.0)
        qbar_min = config.qbar_min_authority_pa
        qbar_full = config.qbar_full_authority_pa
        qbar_high = config.qbar_high_authority_pa
        delta_min = config.delta_max_qbar_min_rad
        delta_high = config.delta_max_qbar_high_rad

        if qbar_full <= qbar_min:
            limit = delta_max_rad
        elif q_dynamic <= qbar_min:
            limit = delta_min
        elif q_dynamic <= qbar_full:
            frac = (q_dynamic - qbar_min) / (qbar_full - qbar_min)
            limit = delta_min + frac * (delta_max_rad - delta_min)
        elif qbar_high <= qbar_full or q_dynamic >= qbar_high:
            limit = delta_high
        else:
            frac = (q_dynamic - qbar_full) / (qbar_high - qbar_full)
            limit = delta_max_rad + frac * (delta_high - delta_max_rad)
        
        limits_deg.append(limit * 180 / np.pi)
    
    limits_deg = np.array(limits_deg)
    
    # Plot standard fins
    for i in range(4):
        plt.plot(times, deltas[:, i], label=f"Fin {i+1}", linewidth=1.5)
        
    # Overlay dynamic deflection limit lines
    plt.plot(times, limits_deg, 'k--', alpha=0.7, label="Deflection Limit", linewidth=1.5)
    plt.plot(times, -limits_deg, 'k--', alpha=0.7, linewidth=1.5)
    
    plt.xlabel("Time (s)")
    plt.ylabel("Deflection (deg)")
    plt.title("Control Phase: Fin Actuation with Authority Limits")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "fin_actuation.png"))
    if not keep_open:
        plt.close(fig)


def _plot_attitude_euler(times, ctrl_history, out_dir, keep_open=False):
    fig = plt.figure(figsize=(10, 6))
    eulers = np.array([utils.quaternion_to_euler(s["attitude_quaternion"]) for s in ctrl_history]) * 180 / np.pi
    
    # Extract reference Euler angles from s["q_ref"]
    eulers_ref = []
    for s in ctrl_history:
        q_ref = s.get("q_ref")
        if q_ref is not None and not np.any(np.isnan(q_ref)):
            eulers_ref.append(utils.quaternion_to_euler(q_ref))
        else:
            eulers_ref.append((np.nan, np.nan, np.nan))
    eulers_ref = np.array(eulers_ref) * 180 / np.pi

    # Plot Pitch (phi) - index 0 (X-rotation)
    plt.plot(times, eulers[:, 0], label="Pitch Obtained", color="C1", linewidth=1.5)
    plt.plot(times, eulers_ref[:, 0], "--", label="Pitch Required", color="C1", alpha=0.7, linewidth=1.5)
    
    # Plot Yaw (theta) - index 1 (Y-rotation)
    plt.plot(times, eulers[:, 1], label="Yaw Obtained", color="C2", linewidth=1.5)
    plt.plot(times, eulers_ref[:, 1], "--", label="Yaw Required", color="C2", alpha=0.7, linewidth=1.5)
    
    # Plot Roll (psi) - index 2 (Z-rotation)
    plt.plot(times, eulers[:, 2], label="Roll Obtained", color="C0", linewidth=1.5)
    plt.plot(times, eulers_ref[:, 2], "--", label="Roll Required", color="C0", alpha=0.7, linewidth=1.5)
    
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Control Phase: Rocket Attitude Tracking (Euler ZYX)")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "attitude_euler.png"))
    if not keep_open:
        plt.close(fig)


def _plot_body_rates(times, ctrl_history, out_dir, keep_open=False):
    fig = plt.figure(figsize=(10, 6))
    omega = np.array([s["body_rates_rad_s"] for s in ctrl_history]) * 180 / np.pi
    plt.plot(times, omega[:, 0], label="ωx (Pitch rate)", color="C1")
    plt.plot(times, omega[:, 1], label="ωy (Yaw rate)", color="C2")
    plt.plot(times, omega[:, 2], label="ωz (Roll rate)", color="C0")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (deg/s)")
    plt.title("Control Phase: Body Angular Rates")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "body_rates.png"))
    if not keep_open:
        plt.close(fig)


def _plot_velocity_per_axis(times, vel_real, vel_ref, out_dir, label_prefix="", keep_open=False):
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ["X (East)", "Y (North)", "Z (Up)"]
    for i in range(3):
        color_map = {0: "C0", 1: "C1", 2: "C2"}
        axes[i].plot(times, vel_real[:, i], label=f"Real {labels[i]}", color=color_map[i], linewidth=1.5)
        axes[i].plot(times, vel_ref[:, i], "r--", label=f"Ref {labels[i]}", alpha=0.7, linewidth=1.5)
        axes[i].set_ylabel("Velocity (m/s)")
        if i == 0:
            if "Control" in label_prefix:
                axes[i].legend(loc="upper right")
            else:
                axes[i].legend(loc="lower right")
        else:
            axes[i].legend(loc="upper right")
        axes[i].grid(True)
    axes[2].set_xlabel("Time (s)")
    fig.suptitle(f"{label_prefix}Per-Axis Linear Velocity Tracking")
    plt.tight_layout(rect=(0, 0.03, 1, 0.95))
    plt.savefig(os.path.join(out_dir, "velocity_per_axis.png"))
    if not keep_open:
        plt.close(fig)


def _plot_cd_vs_mach(max_mach, config, out_dir, keep_open=False):
    import pandas as pd
    drag_df = pd.read_csv(config.drag_path)
    
    # Filter Cd data up to max flight Mach
    filtered_df = drag_df[drag_df['mach'] <= max_mach]
    
    # Guard against empty filtered df if max_mach is very small
    if filtered_df.empty:
        filtered_df = drag_df.iloc[:2]
        
    fig = plt.figure(figsize=(10, 6))
    plt.plot(filtered_df['mach'], filtered_df['cd'], color='navy', linewidth=2, label='Base Drag Coefficient (Cd)')
    
    # Mark max mach point on the curve
    max_cd = float(filtered_df['cd'].iloc[-1])
    actual_max_mach = float(filtered_df['mach'].iloc[-1])
    plt.scatter(actual_max_mach, max_cd, color='red', marker='o', s=60, label=f'Max Flight Mach ({actual_max_mach:.3f})')
    
    plt.xlabel("Mach Number")
    plt.ylabel("Drag Coefficient (Cd)")
    plt.title("Base Drag Coefficient (Cd) vs Mach Number (Up to Max Flight Mach)")
    plt.legend()
    plt.grid(True)
    
    plt.savefig(os.path.join(out_dir, "cd_vs_mach.png"))
    if not keep_open:
        plt.close(fig)


def _plot_gain_evolution(times, ctrl_history, out_dir, config, keep_open=False):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    q_dynamics = np.array([s.get("q_dynamic", 0.0) for s in ctrl_history])
    
    # Recompute q_scale for each timestep
    q_scales = []
    enable_gs = getattr(config, "enable_gain_scheduling", True)
    q_ref_val = getattr(config, "qbar_ref_pa", 21575.1)
    q_min = getattr(config, "q_min_cutoff_pa", 500.0)
    max_scale = getattr(config, "gain_scheduling_max_scale", 50.0)
    
    for q_dyn in q_dynamics:
        if enable_gs:
            q_s = q_ref_val / max(q_dyn, q_min)
            q_s = min(q_s, max_scale)
        else:
            q_s = 1.0
        q_scales.append(q_s)
    q_scales = np.array(q_scales)
    
    # Upper Plot: Dynamic Pressure and q_scale
    color_q = 'tab:blue'
    ax1.plot(times, q_dynamics, color=color_q, linewidth=2, label="Dynamic Pressure (q)")
    ax1.set_ylabel("Dynamic Pressure (Pa)", color=color_q)
    ax1.tick_params(axis='y', labelcolor=color_q)
    ax1.grid(True, alpha=0.3)
    
    ax1_twin = ax1.twinx()
    color_scale = 'tab:orange'
    ax1_twin.plot(times, q_scales, color=color_scale, linestyle='--', linewidth=2, label="Gain Scale Factor (q_scale)")
    ax1_twin.set_ylabel("Gain Scale Factor (q_scale)", color=color_scale)
    ax1_twin.tick_params(axis='y', labelcolor=color_scale)
    
    # Handle legends together
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax1_twin.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc="upper right")
    ax1.set_title("Control Phase: Dynamic Pressure and Gain Scaling Factor")
    
    # Lower Plot: Evolution of active Kp gains
    Kp_att_base = getattr(config, "Kp_attitude", 0.0)
    Kp_roll_base = getattr(config, "Kp_roll", 0.0)
    
    Kp_att_active = Kp_att_base * q_scales
    Kp_roll_active = Kp_roll_base * q_scales
    
    ax2.plot(times, Kp_att_active, label=f"Active Kp_attitude (Base={Kp_att_base:.6f})", color="C2", linewidth=2)
    ax2.plot(times, Kp_roll_active, label=f"Active Kp_roll (Base={Kp_roll_base:.6f})", color="C3", linewidth=2)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Active Proportional Gain (Kp)")
    ax2.legend(loc="upper left")
    ax2.grid(True)
    ax2.set_title("Control Phase: Evolution of Active Proportional Gains")
    
    plt.tight_layout()
    plt.savefig(os.path.join(out_dir, "gain_evolution.png"))
    if not keep_open:
        plt.close(fig)
