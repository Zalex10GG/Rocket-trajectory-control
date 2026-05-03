"""
Trajectory Creator Script
-------------------------
Generates an uncontrolled (passive) rocket trajectory using RocketPy.
Purpose: Create a baseline CSV reference for trajectory tracking.
Execution: uv run trajectory-creator.py
"""

import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rocketpy import Rocket, GenericMotor, Flight
from initial_data import load_initial_case_data
from src.environment_builder import build_environment

# --- EDITABLE CONSTANTS ---
OUTPUT_CSV_PATH = "data/trajectory/uncontrolled.csv"
REFERENCE_HEADER_CSV_PATH = "data/trajectory/vertical.csv"
PLOTS_DIR = "data/trajectory/plots"
INCLINATION_DEG = 90.0
HEADING_DEG = 0.0
MAX_TIME_S = 600.0
TIME_OVERSHOOT = False
VERBOSE = False
THRUST_INTERPOLATION_METHOD = "linear"

def require_key(section, key, context):
    """
    Ensures a key exists in a dictionary (section), otherwise raises a clear error.
    """
    if key not in section:
        raise KeyError(f"Missing required key '{key}' in section '{context}'.")
    return section[key]

def build_passive_rocket(case_data):
    """
    Constructs a passive (uncontrolled) Rocket and its Motor using project assets.
    """
    params = case_data["rocket_params"]
    body = require_key(params, "body", "rocket TOML [body]")
    m_params = require_key(params, "motor", "rocket TOML [motor]")
    
    # 1. Motor Construction
    motor_df = pd.read_csv(case_data["motor_path"], comment='#')
    thrust_data = motor_df.values 
    
    burn_start = require_key(m_params, "burn_time_start_s", "motor")
    burn_end = require_key(m_params, "burn_time_end_s", "motor")

    motor = GenericMotor(
        thrust_source=thrust_data,
        burn_time=(burn_start, burn_end),
        chamber_radius=require_key(m_params, "chamber_radius_m", "motor"),
        chamber_height=require_key(m_params, "chamber_height_m", "motor"),
        chamber_position=require_key(m_params, "chamber_position_m", "motor"),
        propellant_initial_mass=require_key(m_params, "propellant_initial_mass_kg", "motor"),
        nozzle_radius=require_key(m_params, "nozzle_radius_m", "motor"),
        dry_mass=require_key(m_params, "dry_mass_kg", "motor"),
        center_of_dry_mass_position=require_key(m_params, "center_of_dry_mass_position_m", "motor"),
        dry_inertia=(
            require_key(m_params, "dry_inertia_yy_kg_m2", "motor"), 
            require_key(m_params, "dry_inertia_zz_kg_m2", "motor"), 
            require_key(m_params, "dry_inertia_xx_kg_m2", "motor")
        ),
        nozzle_position=require_key(m_params, "nozzle_position_m", "motor"),
        coordinate_system_orientation=require_key(m_params, "coordinate_system_orientation", "motor"),
        interpolation_method=THRUST_INTERPOLATION_METHOD
    )
    
    # 2. Rocket Construction
    # According to RocketPy's Rocket.coordinate_system_orientation='tail_to_nose':
    # - Origin (0) is at the tail.
    # - Positive Z-axis points towards the nose tip.
    # - Positions are measured from the tail.
    # This matches the user-provided TOML convention.
    
    rp_cm_no_motor = require_key(body, "center_of_mass_without_motor_m", "body")

    rocket_kwargs = {
        "radius": require_key(body, "radius_m", "body"),
        "mass": require_key(body, "dry_mass_kg", "body"),
        "inertia": (
            require_key(body, "inertia_yy_kg_m2", "body"), 
            require_key(body, "inertia_zz_kg_m2", "body"), 
            require_key(body, "inertia_xx_kg_m2", "body")
        ), 
        "power_off_drag": case_data["drag_path"],
        "power_on_drag": case_data["drag_path"],
        "center_of_mass_without_motor": rp_cm_no_motor,
        "coordinate_system_orientation": require_key(body, "coordinate_system_orientation", "body")
    }
        
    rocket = Rocket(**rocket_kwargs)
    
    # Motor coordinate-system origin is placed from TOML data. For the current
    # GenericMotor definition, nozzle_position_m=0 means the nozzle/motor origin
    # is at the tail-referenced rocket origin.
    motor_position = require_key(m_params, "nozzle_position_m", "motor")
    rocket.add_motor(motor, position=motor_position) 
    
    # 3. Add Aerodynamic Surfaces
    nose = require_key(params, "nosecone", "rocket TOML [nosecone]")
    
    # RocketPy add_nose expects the nose tip coordinate in the rocket coordinate
    # system. The TOML keeps nosecone.position_m as a tail-to-nose offset from
    # the nose tip reference, so position_m=0 maps to the full body length.
    body_length = require_key(body, "length_m", "body")
    nose_length = require_key(nose, "length_m", "nosecone")
    nose_position = body_length - require_key(nose, "position_m", "nosecone")
    
    rocket.add_nose(
        length=nose_length,
        kind=require_key(nose, "kind", "nosecone"),
        position=nose_position,
        base_radius=require_key(nose, "base_radius_m", "nosecone"),
    )
    
    # 4. Add Base Fins (Aero stability)
    f = require_key(params, "fins", "rocket TOML [fins]")
    # In tail_to_nose, position is measured from the tail.
    fin_position = require_key(f, "position_from_tail_m", "fins")
    fin_kwargs = {
        "n": require_key(f, "count", "fins"),
        "root_chord": require_key(f, "root_chord_m", "fins"),
        "tip_chord": require_key(f, "tip_chord_m", "fins"),
        "span": require_key(f, "span_m", "fins"),
        "position": fin_position,
        "sweep_angle": require_key(f, "sweep_angle_deg", "fins"),
        "cant_angle": require_key(f, "cant_angle_deg", "fins"),
    }
    
    print(f"\nDIAGNOSTIC - RocketPy Positions (Origin=Tail, Orientation=tail_to_nose):")
    print(f"  Body Length: {require_key(body, 'length_m', 'body')} m")
    print(f"  Motor Position: {motor_position} m")
    print(f"  Fins Position: {fin_position} m")
    print(f"  Nose Tip Position: {nose_position} m")
    print(f"  Nose Base Position: {nose_position - nose_length} m")
    print(f"  Nose Base Radius: {require_key(nose, 'base_radius_m', 'nosecone')} m")
    print(f"  Center of Mass (no motor): {rp_cm_no_motor} m\n")
         
    rocket.add_trapezoidal_fins(**fin_kwargs)
    
    return rocket, motor



def save_plot(plot_func, filename):
    """
    Executes a RocketPy plotting function, suppresses interactive windows,
    and saves the figure to PLOTS_DIR.
    """
    os.makedirs(PLOTS_DIR, exist_ok=True)
    path = os.path.join(PLOTS_DIR, filename)
    
    # Close any existing figures to start clean
    plt.close('all')
    
    original_show = plt.show
    plt.show = lambda *args, **kwargs: None
    try:
        # RocketPy plots often call plt.show() at the end. Suppress it so this
        # script only writes images and never opens interactive windows.
        plot_func()
        
        # RocketPy sometimes creates multiple figures. We save the last active one.
        # If it's a specific set of plots (like motor), it usually produces one main figure.
        if plt.get_fignums():
            plt.savefig(path, bbox_inches='tight')
            if VERBOSE:
                print(f"Saved plot: {path}")
        else:
            print(f"WARNING: No figure generated for {filename}")
            
    except Exception as e:
        print(f"Failed to save plot {filename}: {e}")
    finally:
        plt.show = original_show
        plt.close('all')

def validate_plot(filename):
    """
    Programmatically inspects a PNG file to ensure it's not blank or too small.
    """
    path = os.path.join(PLOTS_DIR, filename)
    if not os.path.exists(path):
        return False, "File does not exist"
    
    file_size = os.path.getsize(path)
    if file_size < 1000: # Very small file is likely not a real plot
        return False, f"File size too small ({file_size} bytes)"
    
    try:
        img = plt.imread(path)
        # Check if all pixels are the same (blank)
        if img.size == 0:
            return False, "Empty image"
        
        # If variance is near 0, it's likely a single color (blank)
        # Note: img is usually (H, W, 4) for RGBA
        import numpy as np
        variance = np.var(img)
        if variance < 1e-6:
            return False, f"Image appears blank (variance: {variance:.2e})"
            
        return True, f"OK ({img.shape[1]}x{img.shape[0]}, {file_size} bytes)"
    except Exception as e:
        return False, f"Error reading image: {e}"

def main():
    """
    Main execution flow for uncontrolled trajectory creation.
    """
    print("--- Trajectory Creator ---")
    
    # Load case data
    case_data = load_initial_case_data()
    
    # Build environment (using elevation from case_data)
    # We pass None for config as build_environment doesn't strictly need it 
    # for the basic setup, but let's check if it's safe.
    # Looking at environment_builder.py, it uses case_data and config (optional/ignored for basic).
    env = build_environment(case_data, None)
    
    # Build rocket
    rocket, motor = build_passive_rocket(case_data)
    
    # Run simulation
    if VERBOSE:
        print(f"Running simulation (inclination={INCLINATION_DEG}, heading={HEADING_DEG})...")
    
    test_flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=case_data["rail_length_m"],
        inclination=INCLINATION_DEG,
        heading=HEADING_DEG,
        max_time=MAX_TIME_S,
        terminate_on_apogee=False,
        time_overshoot=TIME_OVERSHOOT,
        verbose=VERBOSE
    )
    
    # Check if we reached the ground
    if test_flight.t_final >= MAX_TIME_S - 0.01:
        print(f"WARNING: Simulation reached MAX_TIME_S ({MAX_TIME_S}s) before impact!")
    else:
        print(f"Simulation finished at t={test_flight.t_final:.2f}s.")

    # Process results
    # Local ENU starting at launch pad (0,0,0)
    # Positions in RocketPy Flight: x, y, z (Earth-centered or relative to pad depending on coordinate system)
    # By default, RocketPy Flight positions are relative to the launch site.
    # However, 'z' usually includes elevation if not careful.
    # Let's verify: test_flight.z(0) should be 0 if relative to pad.
    # Actually, RocketPy's Flight.z is altitude above sea level if env has elevation.
    # We need ENU (0,0,0) at launch.
    
    z_offset = test_flight.z(0)
    x_offset = test_flight.x(0)
    y_offset = test_flight.y(0)
    
    if VERBOSE:
        print(f"Offsets: x={x_offset:.2f}, y={y_offset:.2f}, z={z_offset:.2f}")

    # Extract time vector and solution
    # In RocketPy 1.12.1, solution_array has [t, x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]
    sol = test_flight.solution_array
    
    # Prepare data for CSV
    data_rows = []
    for row in sol:
        t = row[0]
        data_rows.append([
            t,
            row[1] - x_offset,
            row[2] - y_offset,
            row[3] - z_offset,
            row[4],
            row[5],
            row[6]
        ])
        
    # Read reference header only
    try:
        ref_df_header = pd.read_csv(REFERENCE_HEADER_CSV_PATH, nrows=0)
        header = ref_df_header.columns.tolist()
    except Exception as e:
        print(f"Error reading reference header from {REFERENCE_HEADER_CSV_PATH}: {e}")
        return

    # Validation: Expect exactly 7 columns
    if len(header) != 7:
        print(f"CRITICAL ERROR: Reference header in {REFERENCE_HEADER_CSV_PATH} has {len(header)} columns, expected 7.")
        print(f"Found: {header}")
        return
    
    # Create DataFrame and save
    out_df = pd.DataFrame(data_rows, columns=header)
    
    os.makedirs(os.path.dirname(OUTPUT_CSV_PATH), exist_ok=True)
    out_df.to_csv(OUTPUT_CSV_PATH, index=False)
    
    print(f"Trajectory saved to {OUTPUT_CSV_PATH}")
    print(f"Total rows: {len(out_df)}")
    
    # Use .iloc[row_idx, col_idx] to avoid KeyError with column names
    final_row = out_df.iloc[-1]
    print(f"Final position (local ENU): ({final_row.iloc[1]:.2f}, {final_row.iloc[2]:.2f}, {final_row.iloc[3]:.2f})")

    # Generate and save plots
    print("Generating plots...")
    stem = os.path.splitext(os.path.basename(OUTPUT_CSV_PATH))[0]
    
    # Update PLOTS_DIR to be stem-specific
    global PLOTS_DIR
    PLOTS_DIR = os.path.join("data/trajectory/plots", stem)
    
    plot_files = [
        f"{stem}_motor.png",
        f"{stem}_static_margin.png",
        f"{stem}_rocket.png",
        f"{stem}_trajectory_3d.png"
    ]
    
    save_plot(motor.plots.thrust, plot_files[0])
    save_plot(rocket.plots.static_margin, plot_files[1])
    save_plot(rocket.plots.draw, plot_files[2])
    save_plot(test_flight.plots.trajectory_3d, plot_files[3])
    
    # Verification
    print("Verifying plots...")
    all_ok = True
    for plot_file in plot_files:
        ok, msg = validate_plot(plot_file)
        status = "PASS" if ok else "FAIL"
        print(f"  {plot_file}: [{status}] {msg}")
        if not ok:
            all_ok = False
            
    if all_ok:
        print("All plots generated and verified successfully.")
    else:
        print("Some plots failed verification.")

if __name__ == "__main__":
    main()
