import config as cfg
import initial_data as init
import src.controllers as controllers
import src.environment_builder as env_builder
import src.fin_model as fins
import src.metrics as metrics_mod
import src.plots as plots
import src.reference as reference_mod
import src.rocket_builder as rocket_builder
import src.simulation as sim

def main() -> None:
    print("--- TFG Rebuild V1 ---")
    
    # 1. Configuration and Data Loading
    config = cfg.load_config()
    case_data = init.load_initial_case_data()

    # 2. Loading reference
    print("Loading reference...")
    reference = reference_mod.load_reference_trajectory(config.reference_path)
    
    # 3. Initialize Controller State
    controller = controllers.build_controller(config)

    # 4. Build Components
    print("Building environment...")
    environment = env_builder.build_environment(case_data, config)
    
    # 5. Build Rocket
    print("Building rocket...")
    rocket = rocket_builder.build_rocket(case_data, config, controller)

    # 6. Run Simulation (Fin Control integration)
    print("Starting simulation (Fin Control)...")
    flight_history = sim.simulate_controlled_flight(
        rocket=rocket,
        environment=environment,
        reference=reference,
        controller=controller,
        config=config,
    )

    # 5. Analysis and Results
    print("Computing metrics...")
    metrics = metrics_mod.compute_tracking_metrics(flight_history, reference, config)
    
    print("Generating plots and exporting results...")
    sim.export_results(flight_history, reference, metrics, config)
    
    print("--- Done ---")

if __name__ == "__main__":
    main()
