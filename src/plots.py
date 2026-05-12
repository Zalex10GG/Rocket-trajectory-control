"""
Plot generation for flight simulation results.

Two output subdirectories are produced per run:
- plots/simulation/   : full-flight plots (trajectory, rocket, motor)
- plots/control/       : control-phase-only plots (tracking, fin actuation, etc.)
"""

import os
import matplotlib
matplotlib.use("Agg")
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

    # ------------------------------------------------------------------
    # plots/simulation/  (full flight)
    # ------------------------------------------------------------------
    _plot_trajectory_3d(pos_real_local, pos_ref_local_full, sim_dir)
    _plot_trajectory_2d(pos_real_local, pos_ref_local_full, sim_dir)

    # ------------------------------------------------------------------
    # plots/control/  (control phase only)
    # ------------------------------------------------------------------
    _plot_position_per_axis(ctrl_times, pos_real_ctrl, pos_ref_ctrl, ctrl_dir)
    _plot_tracking_errors(ctrl_times, pos_real_ctrl, pos_ref_ctrl, ctrl_dir)
    _plot_fin_actuation(ctrl_times, ctrl_history, ctrl_dir)
    _plot_attitude_euler(ctrl_times, ctrl_history, ctrl_dir)
    _plot_body_rates(ctrl_times, ctrl_history, ctrl_dir)
    _plot_trajectory_3d(pos_real_ctrl, pos_ref_ctrl, ctrl_dir, label_prefix="Control Phase: ")
    _plot_trajectory_2d(pos_real_ctrl, pos_ref_ctrl, ctrl_dir, label_prefix="Control Phase: ")

    print(f"Plots generated in {output_dir}")


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

def _plot_trajectory_3d(pos_real, pos_ref, out_dir, label_prefix=""):
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
    plt.close()


def _plot_trajectory_2d(pos_real, pos_ref, out_dir, label_prefix=""):
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
    plt.close()


def _plot_position_per_axis(times, pos_real, pos_ref, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ["X (East)", "Y (North)", "Z (Up)"]
    for i in range(3):
        axes[i].plot(times, pos_real[:, i], label=f"Real {labels[i]}")
        axes[i].plot(times, pos_ref[:, i], "r--", label=f"Ref {labels[i]}")
        axes[i].set_ylabel("Position Local (m)")
        axes[i].legend(loc="upper right")
        axes[i].grid(True)
    axes[2].set_xlabel("Time (s)")
    fig.suptitle("Control Phase: Per-Axis Position Tracking")
    plt.tight_layout(rect=(0, 0.03, 1, 0.95))
    plt.savefig(os.path.join(out_dir, "position_per_axis.png"))
    plt.close()


def _plot_tracking_errors(times, pos_real, pos_ref, out_dir):
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
    plt.close()


def _plot_fin_actuation(times, ctrl_history, out_dir):
    plt.figure(figsize=(10, 6))
    deltas = np.array([s["deltas"] for s in ctrl_history]) * 180 / np.pi
    for i in range(4):
        plt.plot(times, deltas[:, i], label=f"Fin {i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Deflection (deg)")
    plt.title("Control Phase: Fin Actuation")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "fin_actuation.png"))
    plt.close()


def _plot_attitude_euler(times, ctrl_history, out_dir):
    plt.figure(figsize=(10, 6))
    eulers = np.array([utils.quaternion_to_euler(s["attitude_quaternion"]) for s in ctrl_history]) * 180 / np.pi
    plt.plot(times, eulers[:, 0], label="Roll (phi)")
    plt.plot(times, eulers[:, 1], label="Pitch (theta)")
    plt.plot(times, eulers[:, 2], label="Yaw (psi)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title("Control Phase: Rocket Attitude (Euler ZYX)")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "attitude_euler.png"))
    plt.close()


def _plot_body_rates(times, ctrl_history, out_dir):
    plt.figure(figsize=(10, 6))
    omega = np.array([s["body_rates_rad_s"] for s in ctrl_history]) * 180 / np.pi
    plt.plot(times, omega[:, 0], label="ωx (Roll rate)")
    plt.plot(times, omega[:, 1], label="ωy (Pitch rate)")
    plt.plot(times, omega[:, 2], label="ωz (Yaw rate)")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (deg/s)")
    plt.title("Control Phase: Body Angular Rates")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "body_rates.png"))
    plt.close()
