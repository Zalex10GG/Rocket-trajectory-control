"""
Trajectory Creator Script (Simplified)
-------------------------------------
Generates an uncontrolled (passive) rocket trajectory using RocketPy.
Baseline CSV reference for trajectory tracking.
"""

import os
import matplotlib
import pandas as pd
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rocketpy import Flight, GenericMotor, Rocket

from initial_data import load_initial_case_data
from src.environment_builder import build_environment
from config import load_config

# --- EDITABLE CONSTANTS ---
OUTPUT_CSV_PATH = "data/trajectory/85degree.csv"
PLOTS_DIR_BASE = "data/trajectory/plots"
INCLINATION_DEG = 85.0
HEADING_DEG = 0.0
MAX_TIME_S = 300.0
TIME_OVERSHOOT = False
VERBOSE = False
THRUST_INTERPOLATION_METHOD = "linear"

def build_passive_rocket(case_data):
    params = case_data["rocket_params"]
    body = params["body"]
    m_params = params["motor"]

    # 1. Motor Construction
    motor_df = pd.read_csv(case_data["motor_path"], comment="#")
    motor = GenericMotor(
        thrust_source=motor_df.values,
        burn_time=(m_params["burn_time_start_s"], m_params["burn_time_end_s"]),
        chamber_radius=m_params["chamber_radius_m"],
        chamber_height=m_params["chamber_height_m"],
        chamber_position=m_params["chamber_position_m"],
        propellant_initial_mass=m_params["propellant_initial_mass_kg"],
        nozzle_radius=m_params["nozzle_radius_m"],
        dry_mass=m_params["dry_mass_kg"],
        center_of_dry_mass_position=m_params["center_of_dry_mass_position_m"],
        dry_inertia=(
            m_params["dry_inertia_yy_kg_m2"],
            m_params["dry_inertia_zz_kg_m2"],
            m_params["dry_inertia_xx_kg_m2"],
        ),
        nozzle_position=m_params["nozzle_position_m"],
        coordinate_system_orientation=m_params["coordinate_system_orientation"],
        interpolation_method=THRUST_INTERPOLATION_METHOD,
    )

    # 2. Rocket Construction
    rocket = Rocket(
        radius=body["radius_m"],
        mass=body["dry_mass_kg"],
        inertia=(
            body["inertia_yy_kg_m2"],
            body["inertia_zz_kg_m2"],
            body["inertia_xx_kg_m2"],
        ),
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=body["center_of_mass_without_motor_m"],
        coordinate_system_orientation=body["coordinate_system_orientation"],
    )
    rocket.add_motor(motor, position=m_params.get("position_m", 0.0))

    # 3. Add Aerodynamic Surfaces
    nose = params["nosecone"]
    rocket.add_nose(
        length=nose["length_m"],
        kind=nose["kind"],
        position=nose["position_m"],
        base_radius=nose["base_radius_m"],
    )

    # 4. Add Base Fins
    f = params["fins"]
    rocket.add_trapezoidal_fins(
        n=f["count"],
        root_chord=f["root_chord_m"],
        tip_chord=f["tip_chord_m"],
        span=f["span_m"],
        position=f["position_from_tail_m"],
        sweep_angle=f["sweep_angle_deg"],
        cant_angle=f["cant_angle_deg"],
    )
    return rocket, motor

def save_plot(plot_func, directory, filename):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, filename)
    plt.close("all")
    original_show = plt.show
    plt.show = lambda *args, **kwargs: None
    try:
        plot_func()
        if plt.get_fignums():
            plt.savefig(path, bbox_inches="tight")
    finally:
        plt.show = original_show
        plt.close("all")

def main():
    print("--- Trajectory Creator (Simplified) ---")
    config = load_config()
    case_data = load_initial_case_data(config)
    env = build_environment(case_data, config)
    rocket, motor = build_passive_rocket(case_data)

    test_flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=case_data["rail_length_m"],
        inclination=INCLINATION_DEG,
        heading=HEADING_DEG,
        max_time=MAX_TIME_S,
        terminate_on_apogee=False,
        time_overshoot=TIME_OVERSHOOT,
        verbose=VERBOSE,
    )

    print(f"Simulation finished at t={test_flight.t_final:.2f}s.")

    # Offset positions to local ENU (0,0,0) at launch
    sol = test_flight.solution_array
    x_off, y_off, z_off = sol[0, 1], sol[0, 2], sol[0, 3]
    
    data_rows = []
    for row in sol:
        data_rows.append([row[0], row[1]-x_off, row[2]-y_off, row[3]-z_off, row[4], row[5], row[6]])

    header = ["time_s", "x_enu_m", "y_enu_m", "z_enu_m",
               "vx_enu_m_s", "vy_enu_m_s", "vz_enu_m_s"]
    out_df = pd.DataFrame(data_rows, columns=header)
    
    os.makedirs(os.path.dirname(OUTPUT_CSV_PATH), exist_ok=True)
    out_df.to_csv(OUTPUT_CSV_PATH, index=False)
    print(f"Trajectory saved to {OUTPUT_CSV_PATH} ({len(out_df)} rows)")

    # Plots
    stem = os.path.splitext(os.path.basename(OUTPUT_CSV_PATH))[0]
    plots_dir = os.path.join(PLOTS_DIR_BASE, stem)
    save_plot(motor.plots.thrust, plots_dir, f"{stem}_motor.png")
    save_plot(rocket.plots.static_margin, plots_dir, f"{stem}_static_margin.png")
    save_plot(rocket.plots.draw, plots_dir, f"{stem}_rocket.png")
    save_plot(test_flight.plots.trajectory_3d, plots_dir, f"{stem}_trajectory_3d.png")
    print(f"Plots saved in {plots_dir}")

if __name__ == "__main__":
    main()
