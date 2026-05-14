"""
TFG Rocket Trajectory Control - Main Entry Point

This script orchestrates the full simulation pipeline:
1. Load configuration and rocket parameters.
2. Build the environment, controller, and rocket model.
3. Run the closed-loop flight simulation.
4. Compute performance metrics and export results.

Usage:
    uv run main.py
"""

import config as cfg
import initial_data as init
import src.controllers as controllers
import src.environment_builder as env_builder
import src.metrics as metrics_mod
import src.reference as reference_mod
import src.rocket_builder as rocket_builder
import src.simulation as sim

def main() -> None:
    print("--- TFG Rocket Control Simulation ---")
    
    # 1. Configuration and Data Loading
    config = cfg.load_config()
    case_data = init.load_initial_case_data(config)

    # 2. Loading reference trajectory
    print(f"Loading reference: {config.reference_path}")
    reference = reference_mod.load_reference_trajectory(config.reference_path)
    
    # 3. Initialize Controller State
    controller = controllers.build_controller(config)

    # 4. Build Environment
    print("Building environment...")
    environment = env_builder.build_environment(case_data, config)
    
    # 5. Build Rocket
    print(f"Building rocket from: {config.rocket_path}")
    rocket, components = rocket_builder.build_rocket(case_data, config, controller)

    # 6. Run Simulation (Closed-loop)
    print("Starting controlled flight simulation...")
    flight_history = sim.simulate_controlled_flight(
        rocket=rocket,
        environment=environment,
        reference=reference,
        controller=controller,
        config=config,
    )

    # 7. Analysis and Results
    print("Computing metrics and generating plots...")
    metrics = metrics_mod.compute_tracking_metrics(flight_history, reference, config, controller_state=controller)
    
    sim.export_results(
        flight_history, reference, metrics, config, case_data,
        rocket=rocket, components=components, controller=controller,
    )
    
    print("--- Simulation Complete ---")

if __name__ == "__main__":
    main()
