import matplotlib.pyplot as plt
import os
import numpy as np
import src.reference as ref_mod
import src.utils as utils

def generate_all_plots(flight_history, reference, metrics, config, output_dir=None):
    """
    Generates and saves performance plots. 
    Improved version for general 3D trajectory tracking.
    """
    if not flight_history:
        print("No flight history to plot.")
        return

    # Base plot directory
    if output_dir:
        run_plots_dir = os.path.join(output_dir, "plots")
    else:
        run_plots_dir = os.path.join(config.results_dir, "latest_plots")
        
    os.makedirs(run_plots_dir, exist_ok=True)
    
    times = np.array([s['time_s'] for s in flight_history])
    
    # Identify control window
    start_idx, end_idx = utils.get_control_window_indices(flight_history)
    ctrl_history = flight_history[start_idx:end_idx+1]
    ctrl_times = times[start_idx:end_idx+1]
    
    # Save directory for latest run plots
    pos_real_local = np.array([s['position_enu_m'] for s in flight_history])
    
    # Sample reference at same times
    ref_states = [ref_mod.sample_reference(reference, t) for t in times]
    pos_ref_local = np.array([r['position_enu_m'] for r in ref_states])

    # Extract control window states
    pos_real_ctrl = pos_real_local[start_idx:end_idx+1]
    pos_ref_ctrl = pos_ref_local[start_idx:end_idx+1]

    # 1. 3D Trajectory Comparison
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_real_local[:, 0], pos_real_local[:, 1], pos_real_local[:, 2], label='Real Trajectory (Local ENU)', color='blue', linewidth=2)
    ax.plot(pos_ref_local[:, 0], pos_ref_local[:, 1], pos_ref_local[:, 2], 'k--', label='Reference (Local ENU)', alpha=0.7)
    
    # Mark start and end
    ax.scatter(pos_real_local[0, 0], pos_real_local[0, 1], pos_real_local[0, 2], color='green', marker='o', s=50, label='Launch')
    ax.scatter(pos_real_local[-1, 0], pos_real_local[-1, 1], pos_real_local[-1, 2], color='red', marker='x', s=50, label='Apogee')
    
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')
    ax.set_title('3D Trajectory Tracking (Local Origin)')
    ax.legend()
    plt.savefig(os.path.join(run_plots_dir, "trajectory_3d.png"))
    plt.close()

    # 2. Per-Axis Position Tracking (Control Phase)
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    labels = ['X (East)', 'Y (North)', 'Z (Up)']
    for i in range(3):
        axes[i].plot(ctrl_times, pos_real_ctrl[:, i], label=f'Real {labels[i]}')
        axes[i].plot(ctrl_times, pos_ref_ctrl[:, i], 'r--', label=f'Ref {labels[i]}')
        axes[i].set_ylabel('Position Local (m)')
        axes[i].legend(loc='upper right')
        axes[i].grid(True)
    axes[2].set_xlabel('Time (s)')
    fig.suptitle('Control Phase: Per-Axis Position Tracking')
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig(os.path.join(run_plots_dir, "position_per_axis.png"))
    plt.close()

    # 3. Tracking Errors (Control Phase)
    errors_ctrl = pos_ref_ctrl - pos_real_ctrl
    error_norm_ctrl = np.linalg.norm(errors_ctrl, axis=1)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    
    # 3a. Norm error
    ax1.plot(ctrl_times, error_norm_ctrl, color='purple', label='3D Distance Error')
    ax1.set_ylabel('Error (m)')
    ax1.set_title('Control Phase: Total Tracking Error (Norm)')
    ax1.legend()
    ax1.grid(True)
    
    # 3b. Per-axis error
    ax2.plot(ctrl_times, errors_ctrl[:, 0], label='Error X')
    ax2.plot(ctrl_times, errors_ctrl[:, 1], label='Error Y')
    ax2.plot(ctrl_times, errors_ctrl[:, 2], label='Error Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (m)')
    ax2.set_title('Control Phase: Per-Axis Tracking Error')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig(os.path.join(run_plots_dir, "tracking_errors.png"))
    plt.close()

    # 4. Fin Deflections (Control Phase)
    plt.figure(figsize=(10, 6))
    deltas_ctrl = np.array([s['deltas'] for s in ctrl_history]) * 180 / np.pi
    for i in range(4):
        plt.plot(ctrl_times, deltas_ctrl[:, i], label=f'Fin {i+1}')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Deflection (deg)')
    plt.title('Control Phase: Fin Actuation')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(run_plots_dir, "fin_actuation.png"))
    plt.close()

    # 5. Attitude (Control Phase)
    plt.figure(figsize=(10, 6))
    eulers_ctrl = np.array([utils.quaternion_to_euler(s['attitude_quaternion']) for s in ctrl_history]) * 180 / np.pi
    
    plt.plot(ctrl_times, eulers_ctrl[:, 0], label='Roll (phi)')
    plt.plot(ctrl_times, eulers_ctrl[:, 1], label='Pitch (theta)')
    plt.plot(ctrl_times, eulers_ctrl[:, 2], label='Yaw (psi)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (deg)')
    plt.title('Control Phase: Rocket Attitude (Euler ZYX)')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(run_plots_dir, "attitude_euler.png"))
    plt.close()

    # 6. Body Rates (Control Phase)
    plt.figure(figsize=(10, 6))
    omega_ctrl = np.array([s['body_rates_rad_s'] for s in ctrl_history]) * 180 / np.pi
    plt.plot(ctrl_times, omega_ctrl[:, 0], label='ωx (Roll rate)')
    plt.plot(ctrl_times, omega_ctrl[:, 1], label='ωy (Pitch rate)')
    plt.plot(ctrl_times, omega_ctrl[:, 2], label='ωz (Yaw rate)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (deg/s)')
    plt.title('Control Phase: Body Angular Rates')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(run_plots_dir, "body_rates.png"))
    plt.close()

    # 7. 2D Projections (Top and Side Views)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # 7a. XY (Top View)
    axes[0].plot(pos_real_local[:, 0], pos_real_local[:, 1], label='Real', color='blue')
    axes[0].plot(pos_ref_local[:, 0], pos_ref_local[:, 1], 'k--', label='Ref', alpha=0.5)
    axes[0].set_xlabel('East (m)')
    axes[0].set_ylabel('North (m)')
    axes[0].set_title('Top View (XY)')
    axes[0].grid(True)
    axes[0].legend()
    axes[0].axis('equal')

    # 7b. XZ (Side View - East/Up)
    axes[1].plot(pos_real_local[:, 0], pos_real_local[:, 2], label='Real', color='blue')
    axes[1].plot(pos_ref_local[:, 0], pos_ref_local[:, 2], 'k--', label='Ref', alpha=0.5)
    axes[1].set_xlabel('East (m)')
    axes[1].set_ylabel('Up (m)')
    axes[1].set_title('Side View (XZ)')
    axes[1].grid(True)
    axes[1].axis('equal')

    # 7c. YZ (Side View - North/Up)
    axes[2].plot(pos_real_local[:, 1], pos_real_local[:, 2], label='Real', color='blue')
    axes[2].plot(pos_ref_local[:, 1], pos_ref_local[:, 2], 'k--', label='Ref', alpha=0.5)
    axes[2].set_xlabel('North (m)')
    axes[2].set_ylabel('Up (m)')
    axes[2].set_title('Profile View (YZ)')
    axes[2].grid(True)
    axes[2].axis('equal')

    plt.tight_layout()
    plt.savefig(os.path.join(run_plots_dir, "trajectory_2d_projections.png"))
    plt.close()

    print(f"Plotting suite updated. 7 analysis plots generated in {run_plots_dir}")
