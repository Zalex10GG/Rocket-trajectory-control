import hashlib
import json
import os
import subprocess
from datetime import datetime

from rocketpy import GenericMotor, GenericSurface, Rocket
from rocketpy.plots.rocket_plots import _RocketPlots
from rocketpy.rocket.aero_surface import Fins

from src.constants import CONTROL_SURFACE_NAME
from src.fin_model import FinAdapter, set_controller_state_ref


class _FixedRocketPlots(_RocketPlots):
    """Patches RocketPy's draw so that fin root chords are drawn even when
    GenericSurface is placed after fins in the aerodynamic surface list.
    """

    def _draw_tubes(self, ax, drawn_surfaces, vis_args):
        radius, last_x = super()._draw_tubes(ax, drawn_surfaces, vis_args)
        for i, d_surface in enumerate(drawn_surfaces):
            surface, position, surf_radius, surf_last_x = d_surface
            if isinstance(surface, Fins) and i != len(drawn_surfaces) - 1:
                x_tube = [position, surf_last_x]
                y_tube = [surf_radius, surf_radius]
                y_tube_neg = [-surf_radius, -surf_radius]
                ax.plot(
                    x_tube,
                    y_tube,
                    color=vis_args["body"],
                    linewidth=vis_args["line_width"],
                )
                ax.plot(
                    x_tube,
                    y_tube_neg,
                    color=vis_args["body"],
                    linewidth=vis_args["line_width"],
                )
        return radius, last_x


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
        commit = (
            subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL
            )
            .decode()
            .strip()
        )
        is_dirty = (
            subprocess.call(["git", "diff", "--quiet"], stderr=subprocess.DEVNULL) != 0
        )
        return {"commit": commit, "dirty": is_dirty}
    except Exception:
        return {"git_not_available": True}


def export_rocket_creation_artifacts(rocket, components, run_dir, config, case_data):
    """
    Exports comprehensive metadata about the rocket and its environment to the run directory.
    """
    # 1. Effective Config (serializable parts)
    config_dict = {
        k: v
        for k, v in config.__dict__.items()
        if isinstance(v, (int, float, str, bool, list, dict))
    }
    with open(os.path.join(run_dir, "effective_config.json"), "w") as f:
        json.dump(config_dict, f, indent=4)

    # 2. Manifest and Hashes
    manifest = {
        "timestamp": datetime.now().isoformat(),
        "git": get_git_metadata(),
        "assets": {
            "rocket_toml": {
                "path": case_data["rocket_path"],
                "hash": get_file_hash(case_data["rocket_path"]),
            },
            "motor_csv": {
                "path": case_data["motor_path"],
                "hash": get_file_hash(case_data["motor_path"]),
            },
            "drag_csv": {
                "path": case_data["drag_path"],
                "hash": get_file_hash(case_data["drag_path"]),
            },
            "reference_csv": {
                "path": config.reference_path,
                "hash": get_file_hash(config.reference_path),
            },
        },
    }
    with open(os.path.join(run_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=4)

    # 3. Copy Rocket TOML for direct reference
    if os.path.exists(case_data["rocket_path"]):
        import shutil

        shutil.copy2(
            case_data["rocket_path"], os.path.join(run_dir, "rocket_definition.toml")
        )

    # 4. Basic Rocket Stats
    artifacts = {
        "rocket": {
            "mass_kg": float(rocket.mass),
            "center_of_mass_without_motor_m": float(
                rocket.center_of_mass_without_motor
            ),
            "radius_m": float(rocket.radius),
        },
        "components": list(components.keys()),
    }
    with open(os.path.join(run_dir, "rocket_artifacts.json"), "w") as f:
        json.dump(artifacts, f, indent=4)


def build_rocket(case_data, config, controller_state):
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
        raise ValueError(
            "Critical error: 'control_actuation' missing in rocket_params. Cannot implement closed-loop."
        )

    actuation = params["control_actuation"]
    geom = params["body"]
    motor_params = params["motor"]

    # Safety check: Zero inertia zz validation
    # inertia_zz_kg_m2 is the moment of inertia about the longitudinal axis.
    # If it is exactly 0.0, roll control calculations and simulation stability might fail.
    inertia_zz = geom.get("inertia_zz_kg_m2", 0.0)
    if inertia_zz <= 0.0:
        raise ValueError(
            f"Rocket {params.get('name', 'unnamed')} has invalid inertia_zz_kg_m2 ({inertia_zz}). "
            "A positive value is required for physical simulation and roll control."
        )

    # 1. Motor Construction
    import pandas as pd

    motor_df = pd.read_csv(case_data["motor_path"], comment="#")
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
        # RocketPy expects (I11, I22, I33) where e3 is the longitudinal (roll) axis.
        # In tail_to_nose coords: I11=Iyy (pitch), I22=Izz (yaw), I33=Ixx (roll/axial).
        # TOML convention: xx=roll(axial), yy=pitch, zz=yaw — matching this mapping.
        dry_inertia=(
            motor_params["dry_inertia_yy_kg_m2"],  # I11 (pitch)
            motor_params["dry_inertia_zz_kg_m2"],  # I22 (yaw)
            motor_params["dry_inertia_xx_kg_m2"],  # I33 (roll/axial)
        ),
        nozzle_position=motor_params["nozzle_position_m"],
        interpolation_method="linear",
        coordinate_system_orientation=motor_params["coordinate_system_orientation"],
    )

    rocket = Rocket(
        radius=geom["radius_m"],
        mass=geom["dry_mass_kg"],
        # RocketPy expects (I11, I22, I33) — see motor dry_inertia comment above.
        inertia=(
            geom["inertia_yy_kg_m2"],
            geom["inertia_zz_kg_m2"],
            geom["inertia_xx_kg_m2"],
        ),
        power_off_drag=case_data["drag_path"],
        power_on_drag=case_data["drag_path"],
        center_of_mass_without_motor=geom["center_of_mass_without_motor_m"],
        coordinate_system_orientation=geom["coordinate_system_orientation"],
    )

    # Place the motor so its origin (nozzle, since nozzle_position=0) sits at the
    # rocket tail. In tail_to_nose the tail is the origin (x=0).
    motor_position = motor_params.get("position_m", 0.0)
    rocket.add_motor(motor, position=motor_position)

    # 2. Add Aerodynamic Surfaces
    # NOTE: add_nose expects the nose-tip coordinate in the rocket coordinate system.
    # TOML stores nosecone.position_m as a tail-to-nose offset (0 = tip at full body length),
    # so we convert: position = body_length - tail_offset.
    # Pass base_radius explicitly so the controlled simulation matches trajectory-creator.py
    # even if the nosecone base radius ever differs from the body radius.
    rocket.add_nose(
        length=params["nosecone"]["length_m"],
        kind=params["nosecone"]["kind"],
        position=params["nosecone"]["position_m"],
        base_radius=params["nosecone"].get("base_radius_m", geom["radius_m"]),
    )

    # 3. Add Controlled GenericSurface
    # Initialize adapter and coefficients
    adapter = FinAdapter(controller_state, actuation)
    coeffs = adapter.get_coefficients_dict()

    # Register the controller state dict as the module-level singleton
    # so that FinAdapter coefficient functions (wrapped in RocketPy Function
    # objects) can read current_deltas even after RocketPy deep-copies
    # the rocket during Flight initialisation.
    set_controller_state_ref(controller_state)

    # Store actuation limits in controller state for the controller to use
    if "delta_max_rad" not in actuation or "delta_dot_max_rad_s" not in actuation:
        raise ValueError(
            "Critical error: 'delta_max_rad' and 'delta_dot_max_rad_s' "
            "must be defined in rocket TOML [control_actuation]."
        )

    controller_state["delta_max_rad"] = actuation["delta_max_rad"]
    controller_state["delta_dot_max_rad_s"] = actuation["delta_dot_max_rad_s"]

    # Propagate actuation parameters to config for controller diagnostics
    # (q-bar scheduling, cD computation) without importing the TOML.
    config._delta_max_rad_from_toml = actuation["delta_max_rad"]
    config._cN_delta_per_rad = actuation.get("cN_delta_per_rad", 9.343586365106)
    config._cy_delta_per_rad = actuation.get("cy_delta_per_rad", 9.343586365106)
    config._k_drag_induced = actuation.get("k_drag_induced", 0.295907824866)

    # Deriving minimum activation height
    config.control_start_min_height_above_launch_m = (
        config.rail_length_m + config.safety_margin_m
    )

    # Base Fins (Aero stability)
    f = params["fins"]

    # Read the center of pressure from the TOML; it was precomputed by
    # rocketpy.TrapezoidalFins.evaluate_center_of_pressure() and stored as a constant.
    cpz = f.get("center_of_pressure_m")
    if cpz is None:
        raise KeyError(
            "Missing 'center_of_pressure_m' in [fins] section of rocket TOML."
        )

    # GenericSurface.center_of_pressure is defined in the surface's local coords.
    # To align both physics and the rocket diagram, we place the GenericSurface
    # origin at the fin's absolute center of pressure and set the local CP to
    # (0, 0, 0). In fin local coords cpz is positive downwards (towards the tail),
    # so the absolute CP in tail_to_nose rocket coords is:
    #   position_from_tail_m - cpz
    control_surface = GenericSurface(
        reference_area=actuation["reference_area_m2"],
        reference_length=actuation["reference_length_m"],
        coefficients=coeffs,
        center_of_pressure=(0, 0, 0),
        name=CONTROL_SURFACE_NAME,
    )

    # add_surfaces positions the surface's origin at the given rocket coord.
    # Because local CP is (0,0,0), the origin is the aerodynamic center and the
    # plotted point will appear at the correct CP location.
    fin_cp_position = f["position_from_tail_m"] - cpz
    rocket.add_surfaces(control_surface, fin_cp_position)

    # 4. Parachute
    main_parachute = None
    if "parachute" in params:
        p_params = params["parachute"]
        main_parachute = rocket.add_parachute(
            name=p_params.get("name", "Main"),
            cd_s=p_params["cd_s"],
            trigger=p_params.get("trigger", "apogee"),
        )

    # 5. Base Fins (Aero stability)
    # Leon 2 base fins are NOT controlled, they provide passive stability.
    # position is root chord leading edge (highest point) in tail_to_nose coords.
    base_fins = rocket.add_trapezoidal_fins(
        n=f["count"],
        root_chord=f["root_chord_m"],
        tip_chord=f["tip_chord_m"],
        span=f["span_m"],
        position=f["position_from_tail_m"],
        sweep_length=None,
        sweep_angle=f["sweep_angle_deg"],
        cant_angle=f["cant_angle_deg"],
    )

    rocket.plots = _FixedRocketPlots(rocket)

    components = {
        "motor": motor,
        "nose": rocket.aerodynamic_surfaces[0],  # Nose is usually first
        "control_fins": control_surface,
        "base_fins": base_fins,
        "parachute": main_parachute,
    }
    return rocket, components
