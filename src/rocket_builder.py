import numpy as np
import toml
import os
from rocketpy import Rocket, SolidMotor, GenericSurface
from src.fin_model import FinAdapter

def build_rocket(case_data, config, controller_state):
    """
    Constructs the Rocket and its Motor using actual assets.
    """
    params = case_data["rocket_params"]
    if "control_actuation" not in params:
        raise ValueError("Critical error: 'control_actuation' missing in rocket_params. Cannot implement closed-loop.")
    
    actuation = params["control_actuation"]
    geom = params["geometry"]
    
    # 1. Motor Construction
    import pandas as pd
    motor_df = pd.read_csv(case_data["motor_path"], comment='#')
    thrust_data = motor_df.values 
    
    motor = SolidMotor(
        thrust_source=thrust_data,
        dry_mass=2.0,
        dry_inertia=(0.1, 0.1, 0.01),
        nozzle_radius=0.033,
        throat_radius=0.011,
        grain_number=3,
        grain_density=1020,
        grain_outer_radius=0.033,
        grain_initial_inner_radius=0.015,
        grain_initial_height=0.12,
        grain_separation=0.005,
        grains_center_of_mass_position=0.3,
        center_of_dry_mass_position=0.3,
        interpolation_method='linear',
        coordinate_system_orientation='nozzle_to_combustion_chamber'
    )
    
    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        inertia=(geom["inertia_yy_kg_m2"], geom["inertia_zz_kg_m2"], geom["inertia_xx_kg_m2"]), 
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=0,
        coordinate_system_orientation='tail_to_nose'
    )
    
    rocket.add_motor(motor, position=-geom["length_m"]) 
    
    # 3. Add Aerodynamic Surfaces
    rocket.add_nose(length=0.5, kind="vonKarman", position=0)
    
    # 4. Add Controlled GenericSurface
    # Initialize adapter and coefficients
    adapter = FinAdapter(controller_state, actuation)
    coeffs = adapter.get_coefficients_dict()
    
    control_surface = GenericSurface(
        reference_area=actuation["reference_area_m2"],
        reference_length=actuation["reference_length_m"],
        coefficients=coeffs,
        name="Control Fins"
    )
    # Position in RocketPy coordinate system (tail_to_nose: 0 is tail)
    # GenericSurface CP is relative to rocket coordinate system if added via add_surfaces
    # leon_2.toml: fin_aerodynamic_center_x_m = 0.7 (from nose tip)
    # nose tip is at 0, so position is -0.7
    rocket.add_surfaces(control_surface, -actuation["fin_aerodynamic_center_x_m"])

    # Parachute
    rocket.add_parachute(
        name='Main',
        cd_s=10.0,
        trigger='apogee'
    )
    
    # Base Fins (Aero stability)
    f = params["fins"]
    # Leon 2 base fins are NOT controlled, they provide passive stability.
    rocket.add_trapezoidal_fins(
        n=f["count"],
        root_chord=f["root_chord_m"],
        tip_chord=f["tip_chord_m"],
        span=f["span_m"],
        position=-f["position_from_tail_m"], # Relative to tail tip? No, add_trapezoidal_fins uses position.
        sweep_length=f["sweep_length_m"],
        cant_angle=f["cant_angle_deg"]
    )
    
    return rocket
