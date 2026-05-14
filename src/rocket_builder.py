"""
Builds the RocketPy Rocket object and its components.

Handles the integration of aerodynamic surfaces, motors, and the 
custom GenericSurface used for active fin control.
"""

import os
import json
import pandas as pd
from datetime import datetime
from rocketpy import GenericMotor, GenericSurface, Rocket
from rocketpy.plots.rocket_plots import _RocketPlots
from rocketpy.rocket.aero_surface import Fins

from src.constants import CONTROL_SURFACE_NAME
from src.fin_model import FinAdapter, set_controller_state_ref

class _FixedRocketPlots(_RocketPlots):
    """
    Patches RocketPy's draw to ensure fin root chords are visible in diagrams.
    """
    def _draw_tubes(self, ax, drawn_surfaces, vis_args):
        radius, last_x = super()._draw_tubes(ax, drawn_surfaces, vis_args)
        for i, d_surface in enumerate(drawn_surfaces):
            surface, position, surf_radius, surf_last_x = d_surface
            if isinstance(surface, Fins) and i != len(drawn_surfaces) - 1:
                ax.plot([position, surf_last_x], [surf_radius, surf_radius], color=vis_args["body"], linewidth=vis_args["line_width"])
                ax.plot([position, surf_last_x], [-surf_radius, -surf_radius], color=vis_args["body"], linewidth=vis_args["line_width"])
        return radius, last_x

def export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data):
    """Saves metadata and configuration artifacts for the current run."""
    # Effective Config
    config_dict = {k: v for k, v in config.__dict__.items() if isinstance(v, (int, float, str, bool, list, dict))}
    with open(os.path.join(run_dir, "effective_config.json"), "w") as f:
        json.dump(config_dict, f, indent=4)

    # Copy Rocket TOML
    import shutil
    shutil.copy2(config.rocket_path, os.path.join(run_dir, "rocket_definition.toml"))

    # Basic Rocket Stats
    artifacts = {
        "rocket": {
            "mass_kg": float(rocket.mass),
            "radius_m": float(rocket.radius),
        },
        "components": list(components.keys()),
    }
    with open(os.path.join(run_dir, "rocket_artifacts.json"), "w") as f:
        json.dump(artifacts, f, indent=4)

def build_rocket(case_data, config, controller_state):
    """
    Constructs the Rocket and its Motor from TOML and CSV assets.
    
    WARNING: RocketPy 'tail_to_nose' orientation:
    - Origin (0) is at the tail.
    - Positive axis points towards the nose tip.
    """
    params = case_data["rocket_params"]
    actuation = params["control_actuation"]
    geom = params["body"]
    motor_params = params["motor"]

    # 1. Motor Construction
    motor_df = pd.read_csv(case_data["motor_path"], comment="#")
    motor = GenericMotor(
        thrust_source=motor_df.values,
        burn_time=(motor_params["burn_time_start_s"], motor_params["burn_time_end_s"]),
        chamber_radius=motor_params["chamber_radius_m"],
        chamber_height=motor_params["chamber_height_m"],
        chamber_position=motor_params["chamber_position_m"],
        propellant_initial_mass=motor_params["propellant_initial_mass_kg"],
        nozzle_radius=motor_params["nozzle_radius_m"],
        dry_mass=motor_params["dry_mass_kg"],
        center_of_dry_mass_position=motor_params["center_of_dry_mass_position_m"],
        dry_inertia=(
            motor_params["dry_inertia_yy_kg_m2"],
            motor_params["dry_inertia_zz_kg_m2"],
            motor_params["dry_inertia_xx_kg_m2"],
        ),
        nozzle_position=motor_params["nozzle_position_m"],
        interpolation_method="linear",
        coordinate_system_orientation=motor_params["coordinate_system_orientation"],
    )

    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        inertia=(geom["inertia_yy_kg_m2"], geom["inertia_zz_kg_m2"], geom["inertia_xx_kg_m2"]),
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=geom["center_of_mass_without_motor_m"],
        coordinate_system_orientation=geom["coordinate_system_orientation"],
    )
    rocket.add_motor(motor, position=motor_params.get("position_m", 0.0))

    # 2. Add Aerodynamic Surfaces
    rocket.add_nose(
        length=params["nosecone"]["length_m"],
        kind=params["nosecone"]["kind"],
        position=params["nosecone"]["position_m"],
        base_radius=params["nosecone"].get("base_radius_m", geom["radius_m"]),
    )

    # 3. Add Controlled GenericSurface
    adapter = FinAdapter(controller_state, actuation)
    set_controller_state_ref(controller_state)

    # Populate controller limits from TOML
    controller_state["delta_max_rad"] = actuation["delta_max_rad"]
    controller_state["delta_dot_max_rad_s"] = actuation["delta_dot_max_rad_s"]

    # Derive min activation height
    config.control_start_min_height_above_launch_m = config.rail_length_m + config.safety_margin_m

    control_surface = GenericSurface(
        reference_area=actuation["reference_area_m2"],
        reference_length=actuation["reference_length_m"],
        coefficients=adapter.get_coefficients_dict(),
        center_of_pressure=(0, 0, 0),
        name=CONTROL_SURFACE_NAME,
    )

    f = params["fins"]
    fin_cp_position = f["position_from_tail_m"] - f["center_of_pressure_m"]
    rocket.add_surfaces(control_surface, fin_cp_position)

    # 4. Parachute
    main_parachute = None
    if "parachute" in params:
        p = params["parachute"]
        main_parachute = rocket.add_parachute(name=p.get("name", "Main"), cd_s=p["cd_s"], trigger=p.get("trigger", "apogee"))

    # 5. Base Fins (Passive)
    base_fins = rocket.add_trapezoidal_fins(
        n=f["count"], root_chord=f["root_chord_m"], tip_chord=f["tip_chord_m"],
        span=f["span_m"], position=f["position_from_tail_m"],
        sweep_angle=f["sweep_angle_deg"], cant_angle=f["cant_angle_deg"],
    )

    rocket.plots = _FixedRocketPlots(rocket)
    components = {"motor": motor, "nose": rocket.aerodynamic_surfaces[0], "control_fins": control_surface, "base_fins": base_fins, "parachute": main_parachute}
    return rocket, components
