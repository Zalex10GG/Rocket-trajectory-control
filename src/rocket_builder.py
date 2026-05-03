import os
import json
import warnings
import hashlib
import subprocess
from datetime import datetime
from rocketpy import Rocket, GenericMotor, GenericSurface
from src.fin_model import FinAdapter
from src.constants import CONTROL_SURFACE_NAME

def get_file_hash(filepath):
    """Computes SHA256 hash of a file."""
    if not os.path.exists(filepath):
        return "not_found"
    sha256_hash = hashlib.sha256()
    with open(filepath, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()

def get_git_metadata():
    """Retrieves basic git metadata."""
    try:
        commit = subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
        is_dirty = subprocess.call(["git", "diff", "--quiet"], stderr=subprocess.DEVNULL) != 0
        return {"commit": commit, "dirty": is_dirty}
    except Exception:
        return {"git_not_available": True}

def export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data):
    """
    Exports comprehensive metadata about the rocket and its environment to the run directory.
    """
    # 1. Effective Config (serializable parts)
    config_dict = {k: v for k, v in config.__dict__.items() if isinstance(v, (int, float, str, bool, list, dict))}
    with open(os.path.join(run_dir, "effective_config.json"), "w") as f:
        json.dump(config_dict, f, indent=4)

    # 2. Manifest and Hashes
    manifest = {
        "timestamp": datetime.now().isoformat(),
        "git": get_git_metadata(),
        "assets": {
            "rocket_toml": {"path": case_data["rocket_path"], "hash": get_file_hash(case_data["rocket_path"])},
            "motor_csv": {"path": case_data["motor_path"], "hash": get_file_hash(case_data["motor_path"])},
            "drag_csv": {"path": case_data["drag_path"], "hash": get_file_hash(case_data["drag_path"])},
            "reference_csv": {"path": config.reference_path, "hash": get_file_hash(config.reference_path)},
        }
    }
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=4)

    # 3. Copy Rocket TOML for direct reference
    if os.path.exists(case_data["rocket_path"]):
        import shutil
        shutil.copy2(case_data["rocket_path"], os.path.join(run_dir, "rocket_definition.toml"))

    # 4. Basic Rocket Stats
    artifacts = {
        "rocket": {
            "mass_kg": float(rocket.mass),
            "center_of_mass_without_motor_m": float(rocket.center_of_mass_without_motor),
            "radius_m": float(rocket.radius),
        },
        "components": list(components.keys())
    }
    with open(os.path.join(run_dir, "rocket_artifacts.json"), "w") as f:
        json.dump(artifacts, f, indent=4)

def build_rocket(case_data, config, controller_state, return_components=False):
    """
    Constructs the Rocket and its Motor using actual assets.

    Args:
        case_data (dict): Dictionary containing rocket parameters and asset paths.
        config (Config): Configuration object.
        controller_state (dict): Dictionary to store controller-related state.
        return_components (bool, optional): If True, returns a dictionary of components. Defaults to False.

    Returns:
        Rocket: The constructed Rocket object.
        dict (optional): Dictionary of components if return_components is True.
    """
    params = case_data["rocket_params"]
    if "control_actuation" not in params:
        raise ValueError("Critical error: 'control_actuation' missing in rocket_params. Cannot implement closed-loop.")
    
    actuation = params["control_actuation"]
    geom = params["body"]
    motor_params = params["motor"]

    # Safety check: Zero inertia zz warning
    if geom.get("inertia_zz_kg_m2", 0.0) == 0.0:
        warnings.warn(f"Rocket {params.get('name', 'unnamed')} has inertia_zz_kg_m2 = 0.0. Roll control might be physically unrealistic.")
    
    # 1. Motor Construction
    import pandas as pd
    motor_df = pd.read_csv(case_data["motor_path"], comment='#')
    thrust_data = motor_df.values 
    
    motor = GenericMotor(
        thrust_source=thrust_data,
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
            motor_params["dry_inertia_xx_kg_m2"]
        ),
        nozzle_position=motor_params["nozzle_position_m"],
        interpolation_method='linear',
        coordinate_system_orientation=motor_params["coordinate_system_orientation"]
    )
    
    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        inertia=(geom["inertia_yy_kg_m2"], geom["inertia_zz_kg_m2"], geom["inertia_xx_kg_m2"]), 
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=geom["center_of_mass_without_motor_m"],
        coordinate_system_orientation=geom["coordinate_system_orientation"]
    )
    
    rocket.add_motor(motor, position=-geom["length_m"]) 
    
    # 2. Add Aerodynamic Surfaces
    rocket.add_nose(
        length=params["nosecone"]["length_m"],
        kind=params["nosecone"]["kind"],
        position=params["nosecone"]["position_m"]
    )
    
    # 3. Add Controlled GenericSurface
    # Initialize adapter and coefficients
    adapter = FinAdapter(controller_state, actuation)
    coeffs = adapter.get_coefficients_dict()
    
    # Store actuation limits in controller state for the controller to use
    if "delta_max_rad" not in actuation or "delta_dot_max_rad_s" not in actuation:
        raise ValueError("Critical error: 'delta_max_rad' and 'delta_dot_max_rad_s' "
                         "must be defined in rocket TOML [control_actuation].")
    
    controller_state["delta_max_rad"] = actuation["delta_max_rad"]
    controller_state["delta_dot_max_rad_s"] = actuation["delta_dot_max_rad_s"]
    
    # Deriving minimum activation height
    config.control_start_min_height_above_launch_m = config.rail_length_m + config.safety_margin_m

    control_surface = GenericSurface(
        reference_area=actuation["reference_area_m2"],
        reference_length=actuation["reference_length_m"],
        coefficients=coeffs,
        name=CONTROL_SURFACE_NAME
    )
    
    # Base Fins (Aero stability)
    f = params["fins"]
    
    # Position in RocketPy coordinate system (tail_to_nose: 0 is tail)
    rocket.add_surfaces(control_surface, -f["position_from_tail_m"])

    # 4. Parachute
    main_parachute = rocket.add_parachute(
        name='Main',
        cd_s=10.0,
        trigger='apogee'
    )
    
    # 5. Base Fins (Aero stability)
    # Leon 2 base fins are NOT controlled, they provide passive stability.
    base_fins = rocket.add_trapezoidal_fins(
        n=f["count"],
        root_chord=f["root_chord_m"],
        tip_chord=f["tip_chord_m"],
        span=f["span_m"],
        position=-f["position_from_tail_m"], 
        sweep_length=None,
        sweep_angle=f["sweep_angle_deg"],
        cant_angle=f["cant_angle_deg"]
    )

    if return_components:
        components = {
            "motor": motor,
            "nose": rocket.aerodynamic_surfaces[0], # Nose is usually first
            "control_fins": control_surface,
            "base_fins": base_fins,
            "parachute": main_parachute
        }
        return rocket, components
    
    return rocket
