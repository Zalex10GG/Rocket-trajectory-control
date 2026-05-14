"""
Calculate rocket control aerodynamic coefficients (Simplified).

This script calculates incremental force coefficients (cN, cy) and induced drag
due to control fin deflection based on rocket geometry.

Adjust parameters at the top of the script.
"""

import math
import os
import re

import toml

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
TOML_PATH = "data/rockets/leon_2.toml"
MACH = 0.0  # Design Mach number (0.0 to <1.0)
OSWALD_EFFICIENCY = 0.7  # Oswald efficiency factor (e)
PRECISION = 12  # Decimal precision for rounding
UPDATE_TOML = False  # Set to True to save results back to the TOML file

# ---------------------------------------------------------------------------
# Aerodynamic Formulas
# ---------------------------------------------------------------------------


def calculate_fin_area(root_chord: float, tip_chord: float, span: float) -> float:
    return 0.5 * (root_chord + tip_chord) * span


def calculate_midchord_sweep(
    root_chord: float, tip_chord: float, span: float, sweep_angle_deg: float
) -> float:
    sweep_length = span * math.tan(math.radians(sweep_angle_deg))
    delta_x_midchord = sweep_length + 0.5 * tip_chord - 0.5 * root_chord
    return math.atan2(delta_x_midchord, span)


def calculate_beta(mach: float) -> float:
    return math.sqrt(1 - mach**2)


def calculate_cn_delta_isolated(
    span: float, a_ref: float, a_fin: float, beta: float, gamma_c: float
) -> float:
    cos_gamma_c = math.cos(gamma_c)
    term_root = math.sqrt(1 + (beta * (span**2) / (a_fin * cos_gamma_c)) ** 2)
    return (2 * math.pi * (span**2) / a_ref) / (1 + term_root)


def calculate_interference_ktb(radius_body: float, span: float) -> float:
    return 1 + radius_body / (span + radius_body)


def calculate_k_drag_induced(
    a_fin: float, span: float, oswald_efficiency: float
) -> float:
    return a_fin / (2 * math.pi * (span**2) * oswald_efficiency)


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------


def format_float_toml(value: float, precision: int) -> str:
    if value == 0.0:
        return "0.0"
    return f"{value:.{precision}f}".rstrip("0").rstrip(".")


def update_toml_preserving_format(file_path: str, updates: dict, precision: int):
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    new_lines, in_section, found_section, updated_keys = [], False, False, set()
    section_pattern = re.compile(r"^\s*\[([^\]]+)\]")
    key_pattern = re.compile(r"^\s*(\w+)\s*=")
    for line in lines:
        section_match = section_pattern.match(line)
        if section_match:
            if in_section:
                for key, value in updates.items():
                    if key not in updated_keys:
                        new_lines.append(
                            f"{key} = {format_float_toml(value, precision)}\n"
                        )
                        updated_keys.add(key)
            in_section = section_match.group(1) == "control_actuation"
            if in_section:
                found_section = True
        if in_section:
            key_match = key_pattern.match(line)
            if key_match:
                key = key_match.group(1)
                if key in updates:
                    indent = line[: line.find(key)]
                    new_lines.append(
                        f"{indent}{key} = {format_float_toml(updates[key], precision)}\n"
                    )
                    updated_keys.add(key)
                    continue
        new_lines.append(line)
    if in_section:
        for key, value in updates.items():
            if key not in updated_keys:
                new_lines.append(f"{key} = {format_float_toml(value, precision)}\n")
                updated_keys.add(key)
    if not found_section:
        if new_lines and not new_lines[-1].endswith("\n"):
            new_lines[-1] += "\n"
        if new_lines and new_lines[-1].strip():
            new_lines.append("\n")
        new_lines.append("[control_actuation]\n")
        for key, value in updates.items():
            new_lines.append(f"{key} = {format_float_toml(value, precision)}\n")
    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_lines)


# ---------------------------------------------------------------------------
# Main Execution
# ---------------------------------------------------------------------------


def main():
    if not os.path.exists(TOML_PATH):
        print(f"Error: File not found: {TOML_PATH}")
        return

    try:
        with open(TOML_PATH, "r", encoding="utf-8") as f:
            data = toml.load(f)

        r_t = data["body"]["radius_m"]
        fins = data["fins"]
        control = data.get("control_actuation", {})

        a_fin = calculate_fin_area(
            fins["root_chord_m"], fins["tip_chord_m"], fins["span_m"]
        )
        a_ref = control.get("reference_area_m2", math.pi * (r_t**2))
        gamma_c = calculate_midchord_sweep(
            fins["root_chord_m"],
            fins["tip_chord_m"],
            fins["span_m"],
            fins["sweep_angle_deg"],
        )
        beta = calculate_beta(MACH)
        cn_delta_single = calculate_cn_delta_isolated(
            fins["span_m"], a_ref, a_fin, beta, gamma_c
        )
        k_tb = calculate_interference_ktb(r_t, fins["span_m"])

        cn_delta_total = 2 * cn_delta_single * k_tb
        k_drag_induced = calculate_k_drag_induced(
            a_fin, fins["span_m"], OSWALD_EFFICIENCY
        )

        print(f"\n--- Control Coefficients for {TOML_PATH} ---")
        print(f"Mach: {MACH} | Oswald e: {OSWALD_EFFICIENCY}")
        print(f"cN_delta_per_rad: {cn_delta_total:.{PRECISION}f}")
        print(f"cy_delta_per_rad: {cn_delta_total:.{PRECISION}f}")
        print(f"k_drag_induced:   {k_drag_induced:.{PRECISION}f}")

        if UPDATE_TOML:
            updates = {
                "cN_delta_per_rad": cn_delta_total,
                "cy_delta_per_rad": cn_delta_total,
                "k_drag_induced": k_drag_induced,
            }
            update_toml_preserving_format(TOML_PATH, updates, PRECISION)
            print("\nSuccess: TOML file updated.")
        else:
            print("\nNote: TOML was not updated (UPDATE_TOML = False).")

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
