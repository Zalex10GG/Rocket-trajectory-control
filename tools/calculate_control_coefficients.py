"""
Module to calculate rocket control aerodynamic coefficients.

This script calculates the incremental force coefficients (cN, cy) and
induced drag (k_drag_induced) due to control fin deflection,
based on the rocket geometry defined in a TOML file.

It uses the Diederich theory for low aspect ratio wings and
body-fin interference factors (K_TB).

Main formulas:
1. Trapezoidal fin area: A_fin = 0.5 * (root + tip) * span
2. Midchord sweep angle (Gamma_c): derived from geometry.
3. Lift slope (Diederich):
   cn_delta_single = (2*pi*s^2 / A_ref) / (1 + sqrt(1 + (beta*s^2/(A_fin*cos(Gamma_c)))^2))
4. Interference factor: K_TB = 1 + r_t / (s + r_t)
5. Total coefficients (cruciform): cN_delta = 2 * cn_delta_single * K_TB
"""

import argparse
import math
import os
import re
import toml


def calculate_fin_area(root_chord: float, tip_chord: float, span: float) -> float:
    """Calculate trapezoidal fin area."""
    return 0.5 * (root_chord + tip_chord) * span


def calculate_midchord_sweep(root_chord: float, tip_chord: float, span: float, sweep_length: float) -> float:
    """
    Calculate the midchord sweep angle (Gamma_c).

    For a trapezoidal fin, computes the X displacement of the midchord
    at the tip relative to the midchord at the root.
    """
    delta_x_midchord = sweep_length + 0.5 * tip_chord - 0.5 * root_chord
    return math.atan2(delta_x_midchord, span)


def calculate_beta(mach: float) -> float:
    """Calculate the Prandtl-Glauert compressibility factor."""
    return math.sqrt(1 - mach**2)


def calculate_cn_delta_isolated(span: float, a_ref: float, a_fin: float, beta: float, gamma_c: float) -> float:
    """
    Calculate the normal force slope for a single isolated fin using Diederich.

    Formula: (2*pi*s^2 / A_ref) / (1 + sqrt(1 + (beta*s^2/(A_fin*cos(Gamma_c)))^2))
    """
    cos_gamma_c = math.cos(gamma_c)
    term_root = math.sqrt(1 + (beta * (span**2) / (a_fin * cos_gamma_c))**2)
    return (2 * math.pi * (span**2) / a_ref) / (1 + term_root)


def calculate_interference_ktb(radius_body: float, span: float) -> float:
    """Calculate the body-fin interference factor K_TB."""
    return 1 + radius_body / (span + radius_body)


def calculate_k_drag_induced(a_fin: float, span: float, oswald_efficiency: float) -> float:
    """
    Calculate the induced drag factor k.

    Based on AR = 2 * span^2 / A_fin (effective Aspect Ratio).
    k = 1 / (pi * AR * e)
    """
    return a_fin / (2 * math.pi * (span**2) * oswald_efficiency)


def format_float_toml(value: float, precision: int) -> str:
    """
    Format a floating point number to write it in a TOML file.

    Trailing zeros are removed for readability,
    but `0.0` is preserved to clarify that null coefficients
    are real dimensionless values, not integer counters.
    """
    if value == 0.0:
        return "0.0"
    return f"{value:.{precision}f}".rstrip('0').rstrip('.')


def update_toml_preserving_format(file_path: str, updates: dict, precision: int):
    """
    Update the TOML file replacing only the lines for the specified keys
    within the [control_actuation] section, preserving comments and format.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    new_lines = []
    in_control_section = False
    updated_keys = set()

    # Pattern to identify sections [section]
    section_pattern = re.compile(r'^\s*\[([^\]]+)\]')
    # Pattern to identify key = value
    key_pattern = re.compile(r'^\s*(\w+)\s*=')

    for line in lines:
        section_match = section_pattern.match(line)
        if section_match:
            # If we exit the control section and keys are missing, insert them
            if in_control_section:
                for key, value in updates.items():
                    if key not in updated_keys:
                        formatted_value = format_float_toml(value, precision)
                        new_lines.append(f"{key} = {formatted_value}\n")
                        updated_keys.add(key)
                in_control_section = False

            if section_match.group(1) == "control_actuation":
                in_control_section = True

        if in_control_section:
            key_match = key_pattern.match(line)
            if key_match:
                key = key_match.group(1)
                if key in updates:
                    value = updates[key]
                    formatted_value = format_float_toml(value, precision)
                    # Preserve original indentation if possible
                    indent = line[:line.find(key)]
                    new_lines.append(f"{indent}{key} = {formatted_value}\n")
                    updated_keys.add(key)
                    continue

        new_lines.append(line)

    # Case where the control section is the last in the file
    if in_control_section:
        for key, value in updates.items():
            if key not in updated_keys:
                formatted_value = format_float_toml(value, precision)
                new_lines.append(f"{key} = {formatted_value}\n")
                updated_keys.add(key)

    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)


def main():
    """Main CLI function."""
    parser = argparse.ArgumentParser(description="Calculate control coefficients for RocketPy.")
    parser.add_argument("--toml", type=str, default="data/rockets/leon_2.toml", help="Path to rocket TOML file.")
    parser.add_argument("--mach", type=float, default=0.0, help="Design Mach number (0.0 to <1.0).")
    parser.add_argument("--oswald-efficiency", type=float, default=0.7, help="Oswald efficiency factor (e).")
    parser.add_argument("--dry-run", action="store_true", help="Print results without modifying the file.")
    parser.add_argument("--precision", type=int, default=12, help="Decimal precision for rounding.")

    args = parser.parse_args()

    # Initial validations
    if not (0 <= args.mach < 1):
        parser.error(f"Mach must be between 0 and 1 (exclusive). Given value: {args.mach}")

    if args.oswald_efficiency <= 0:
        parser.error(f"Oswald efficiency must be positive. Given value: {args.oswald_efficiency}")

    if not os.path.exists(args.toml):
        raise SystemExit(f"Error: File not found: {args.toml}")

    # Read data for calculations using standard toml (read-only)
    try:
        with open(args.toml, "r", encoding="utf-8") as f:
            data = toml.load(f)

        body = data["body"]
        fins = data["fins"]
        control = data.get("control_actuation", {})

        r_t = body["radius_m"]
        root_chord = fins["root_chord_m"]
        tip_chord = fins["tip_chord_m"]
        span = fins["span_m"]
        sweep_length = fins.get("sweep_length_m", 0.0)

        # Perform calculations
        a_fin = calculate_fin_area(root_chord, tip_chord, span)

        a_ref_theoretical = math.pi * (r_t**2)
        a_ref = control.get("reference_area_m2", a_ref_theoretical)

        if not math.isclose(a_ref, a_ref_theoretical, rel_tol=0.01):
            print(f"Warning: reference_area_m2 ({a_ref:.6f}) differs from pi*r^2 ({a_ref_theoretical:.6f})")

        gamma_c = calculate_midchord_sweep(root_chord, tip_chord, span, sweep_length)
        beta = calculate_beta(args.mach)
        cn_delta_single = calculate_cn_delta_isolated(span, a_ref, a_fin, beta, gamma_c)
        k_tb = calculate_interference_ktb(r_t, span)

        cn_delta_total = 2 * cn_delta_single * k_tb
        cy_delta_total = cn_delta_total
        k_drag_induced = calculate_k_drag_induced(a_fin, span, args.oswald_efficiency)

        # Summary
        print("\n--- Scientific Calculations Summary ---")
        print(f"A_fin: {a_fin:.6f} m2 | A_ref: {a_ref:.6f} m2")
        print(f"Gamma_c: {math.degrees(gamma_c):.2f}° | Beta: {beta:.4f} | K_TB: {k_tb:.4f}")
        print("-" * 40)
        print(f"cN_delta_per_rad: {cn_delta_total:.{args.precision}f}")
        print(f"cy_delta_per_rad: {cy_delta_total:.{args.precision}f}")
        print(f"k_drag_induced:   {k_drag_induced:.{args.precision}f}")

        updates = {
            "cN_delta_per_rad": cn_delta_total,
            "cy_delta_per_rad": cy_delta_total,
            "k_drag_induced": k_drag_induced,
        }

        if args.dry_run:
            print("\nDry-run: No changes were saved.")
        else:
            update_toml_preserving_format(args.toml, updates, args.precision)
            print(f"\nSuccess: File {args.toml} updated preserving format.")

    except KeyError as e:
        raise SystemExit(f"Error: Missing key {e} in TOML file.")
    except Exception as e:
        raise SystemExit(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
