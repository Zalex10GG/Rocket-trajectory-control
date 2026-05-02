import numpy as np
import toml
import os
from contextlib import redirect_stdout
import io
from rocketpy import Rocket, GenericMotor, GenericSurface
from src.fin_model import FinAdapter

def build_rocket(case_data, config, controller_state, return_components=False):
    """
    Constructs the Rocket and its Motor using actual assets.
    """
    params = case_data["rocket_params"]
    if "control_actuation" not in params:
        raise ValueError("Critical error: 'control_actuation' missing in rocket_params. Cannot implement closed-loop.")
    
    actuation = dict(params["control_actuation"])
    geom = params["body"]
    nose_params = params["nosecone"]
    
    # Values read from TOML
    actual_length = geom["length_m"]
    cg_no_motor = geom["center_of_mass_without_motor_m"]

    # 1. Motor Construction
    import pandas as pd
    motor_df = pd.read_csv(case_data["motor_path"], comment='#')
    thrust_data = motor_df.values

    motor_params = params.get("motor", {})
    motor = GenericMotor(
        thrust_source=thrust_data,
        burn_time=(motor_params.get("burn_time_start_s", 0.1), motor_params.get("burn_time_end_s", 2.129)),
        chamber_radius=motor_params.get("chamber_radius_m", 0.0375),
        chamber_height=motor_params.get("chamber_height_m", 0.486),
        chamber_position=motor_params.get("chamber_position_m", 0.0),
        propellant_initial_mass=motor_params.get("propellant_initial_mass_kg", 1.755),
        nozzle_radius=motor_params.get("nozzle_radius_m", 0.004),
        dry_mass=motor_params.get("dry_mass_kg", 1.586),
        center_of_dry_mass_position=motor_params.get("center_of_dry_mass_position_m", 0.250),
        dry_inertia=(motor_params.get("dry_inertia_xx_kg_m2", 0.001), motor_params.get("dry_inertia_yy_kg_m2", 0.001), motor_params.get("dry_inertia_zz_kg_m2", 0.0)),
        nozzle_position=motor_params.get("nozzle_position_m", 0.0),
        coordinate_system_orientation=motor_params.get("coordinate_system_orientation", "nozzle_to_combustion_chamber"),
    )


    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        inertia=(geom["inertia_yy_kg_m2"], geom["inertia_zz_kg_m2"], geom["inertia_xx_kg_m2"]),  
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=cg_no_motor,
        coordinate_system_orientation='tail_to_nose'
    )
    
    # Motor position: 0 is tail tip, so position is 0
    rocket.add_motor(motor, position=0)  

    # 3. Add Aerodynamic Surfaces
    # Nose tip at actual_length
    nose = rocket.add_nose(
        length=nose_params["length_m"], 
        kind=nose_params["kind"], 
        position=actual_length,
        base_radius=nose_params.get("radius_m", geom["radius_m"])
    )

    f_params = params["fins"]
    is_controlled = f_params.get("controlled", False)

    # Add passive fins (always, for passive stability)
    base_fins = rocket.add_trapezoidal_fins(
        n=f_params["count"],
        root_chord=f_params["root_chord_m"],
        tip_chord=f_params["tip_chord_m"],
        span=f_params["span_m"],
        position=f_params["position_from_tail_m"],
        sweep_length=f_params.get("sweep_length_m", 0.0)
    )

    if is_controlled:
        # GenericSurface is placed at the fins CP
        cp_pos_from_tail = f_params["position_from_tail_m"] - base_fins.cp[2]
        control_pos = cp_pos_from_tail

        # 4. Add Controlled GenericSurface (Incremental control only)
        adapter = FinAdapter(controller_state, actuation)
        coeffs = adapter.get_coefficients_dict()

        # Update config limits with TOML values if present
        if "delta_max_rad" in actuation:
            config.delta_max_rad = actuation["delta_max_rad"]
        if "delta_dot_max_rad_s" in actuation:
            config.delta_dot_max_rad_s = actuation["delta_dot_max_rad_s"]

        control_surface = GenericSurface(
            reference_area=actuation["reference_area_m2"],
            reference_length=actuation["reference_length_m"],
            coefficients=coeffs,
            name="Control Fin Deflection Increment"
        )
        rocket.add_surfaces(control_surface, control_pos)
    else:
        control_surface = None

    # Parachute
    rocket.add_parachute(
        name='Main',
        cd_s=10.0,
        trigger='apogee'
    )
    
    if return_components:
        return rocket, {
            "motor": motor,
            "nose": nose,
            "fins": base_fins,
            "control_fins": control_surface
        }
    return rocket

def export_rocket_creation_artifacts(rocket, components, run_dir, config):
    """
    Saves plots and info of the rocket creation to results/<run_id>/rocket_creation/
    """
    creation_dir = os.path.join(run_dir, "rocket_creation")
    os.makedirs(creation_dir, exist_ok=True)
    
    # 1. Save plots
    import matplotlib.pyplot as plt
    import matplotlib
    # Check if we are using an interactive backend and if we should switch temporarily
    original_backend = matplotlib.get_backend()
    if not config.interactive_rocket_plots:
        try:
            matplotlib.use('Agg', force=True)
        except Exception:
            pass # Fallback to current backend if Agg fails
    
    # helper to handle RocketPy plot calls
    def handle_plot(plot_func, filename):
        """
        Captures plots generated by RocketPy.
        RocketPy often creates its own figures internally and calls plt.show().
        """
        # 1. Record existing figure numbers
        before_fignums = set(plt.get_fignums())
        
        # 2. Call the plot function
        # RocketPy might call plt.show() inside. 
        result = plot_func()
        
        # 3. Identify the figure to save
        fig = None
        if isinstance(result, plt.Figure):
            fig = result
        elif isinstance(result, (list, tuple)) and len(result) > 0 and isinstance(result[0], plt.Figure):
            fig = result[0]
        else:
            # Check for new figures
            after_fignums = set(plt.get_fignums())
            new_fignums = after_fignums - before_fignums
            if new_fignums:
                # Use the most recently created figure
                fig = plt.figure(max(new_fignums))
            else:
                # Fallback to current figure
                fig = plt.gcf()
        
        # 4. Save with robustness
        path = os.path.join(creation_dir, filename)
        
        # Ensure the figure is actually drawn.
        if fig:
            fig.canvas.draw()
            fig.savefig(path, bbox_inches='tight')
        
        # 5. Handle display/cleanup
        if config.interactive_rocket_plots:
            plt.show()
        else:
            if fig:
                plt.close(fig)
            after_fignums = set(plt.get_fignums())
            for fnum in (after_fignums - before_fignums):
                plt.close(fnum)
    
    try:
        # Rocket draw
        handle_plot(rocket.draw, "rocket_draw.png")
        
        # Static margin
        handle_plot(rocket.plots.static_margin, "static_margin.png")
 
        # Motor plot
        if "motor" in components:
            handle_plot(components["motor"].plots.thrust, "motor_thrust.png")
    finally:
        # Restore backend if we changed it
        if not config.interactive_rocket_plots:
            try:
                matplotlib.use(original_backend, force=True)
            except Exception:
                pass

    # 2. Save component info
    info_path = os.path.join(creation_dir, "component_info.txt")
    with open(info_path, "w") as f:
        with redirect_stdout(f):
            print("=== Rocket Info ===")
            rocket.info()
            print("\n=== Nosecone Info ===")
            components["nose"].info()
            
            print("\n=== Passive Fins Info ===")
            if components["fins"]:
                components["fins"].info()
            else:
                print("No passive fins added.")

            print("\n=== Motor Info ===")
            components["motor"].info()
            print("\n=== Control Surface Info ===")
            # GenericSurface doesn't have .info() in all versions, 
            # but let's try or print its repr
            if components["control_fins"]:
                print(f"Name: {components['control_fins'].name}")
                try:
                    components["control_fins"].info()
                except AttributeError:
                    print(components["control_fins"])
            else:
                print("No active control surface added.")
