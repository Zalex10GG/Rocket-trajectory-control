"""
Módulo para calcular los coeficientes de control aerodinámico de un cohete.

Este script calcula los incrementos de coeficientes de fuerza (cN, cy) y drag inducido (k_drag_induced)
debidos a la deflexión de las aletas de control, basándose en la geometría del cohete
definida en un archivo TOML.

Utiliza la teoría de Diederich para alas de baja relación de aspecto y factores de
interferencia cuerpo-aleta (K_TB).

Fórmulas principales:
1. Área de aleta trapezoidal: A_fin = 0.5 * (root + tip) * span
2. Ángulo de flecha al 50% de la cuerda (Gamma_c): Calculado a partir de la geometría.
3. Pendiente de sustentación (Diederich):
   cn_delta_single = (2*pi*s^2 / A_ref) / (1 + sqrt(1 + (beta*s^2/(A_fin*cos(Gamma_c)))^2))
4. Factor de interferencia: K_TB = 1 + r_t / (s + r_t)
5. Coeficientes totales (cruciforme): cN_delta = 2 * cn_delta_single * K_TB
"""

import argparse
import math
import os
import re
import toml

def calcular_area_aleta(root_chord: float, tip_chord: float, span: float) -> float:
    """Calcula el área de una aleta trapezoidal."""
    return 0.5 * (root_chord + tip_chord) * span

def calcular_midchord_sweep(root_chord: float, tip_chord: float, span: float, sweep_length: float) -> float:
    """
    Calcula el ángulo de flecha al 50% de la cuerda (Gamma_c).
    
    Para una aleta trapezoidal, se calcula el desplazamiento en X del punto medio de la cuerda 
    en el tip respecto al punto medio en el root.
    """
    delta_x_midchord = sweep_length + 0.5 * tip_chord - 0.5 * root_chord
    return math.atan2(delta_x_midchord, span)

def calcular_beta(mach: float) -> float:
    """Calcula el factor de compresibilidad de Prandtl-Glauert."""
    return math.sqrt(1 - mach**2)

def calcular_cn_delta_aislada(span: float, a_ref: float, a_fin: float, beta: float, gamma_c: float) -> float:
    """
    Calcula la pendiente de fuerza normal para una sola aleta aislada usando Diederich.
    
    Fórmula: (2*pi*s^2 / A_ref) / (1 + sqrt(1 + (beta*s^2/(A_fin*cos(Gamma_c)))^2))
    """
    cos_gamma_c = math.cos(gamma_c)
    term_raiz = math.sqrt(1 + (beta * (span**2) / (a_fin * cos_gamma_c))**2)
    return (2 * math.pi * (span**2) / a_ref) / (1 + term_raiz)

def calcular_interferencia_ktb(radius_body: float, span: float) -> float:
    """Calcula el factor de interferencia cuerpo-aleta K_TB."""
    return 1 + radius_body / (span + radius_body)

def calcular_k_drag_inducido(a_fin: float, span: float, oswald_efficiency: float) -> float:
    """
    Calcula el factor de drag inducido k.
    
    Basado en AR = 2 * span^2 / A_fin (Aspect Ratio efectivo).
    k = 1 / (pi * AR * e)
    """
    return a_fin / (2 * math.pi * (span**2) * oswald_efficiency)

def formatear_float_toml(value: float, precision: int) -> str:
    """
    Formatea un número de coma flotante para escribirlo en TOML.

    Se eliminan ceros finales en valores no nulos para mantener el archivo legible,
    pero se conserva `0.0` para dejar claro que los coeficientes nulos son reales
    físicos adimensionales, no contadores enteros.
    """
    if value == 0.0:
        return "0.0"
    return f"{value:.{precision}f}".rstrip('0').rstrip('.')


def actualizar_toml_preservando_formato(file_path: str, updates: dict, precision: int):
    """
    Actualiza el archivo TOML reemplazando solo las líneas de las claves especificadas
    dentro de la sección [control_actuation], preservando comentarios y formato.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    new_lines = []
    in_control_section = False
    updated_keys = set()
    
    # Patrón para identificar secciones [seccion]
    section_pattern = re.compile(r'^\s*\[([^\]]+)\]')
    # Patrón para identificar claves key = value
    key_pattern = re.compile(r'^\s*(\w+)\s*=')

    for line in lines:
        section_match = section_pattern.match(line)
        if section_match:
            # Si salimos de la sección de control y faltan claves por actualizar, las insertamos
            if in_control_section:
                for key, value in updates.items():
                    if key not in updated_keys:
                        formatted_value = formatear_float_toml(value, precision)
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
                    formatted_value = formatear_float_toml(value, precision)
                    # Preservar indentación original si es posible
                    indent = line[:line.find(key)]
                    new_lines.append(f"{indent}{key} = {formatted_value}\n")
                    updated_keys.add(key)
                    continue
        
        new_lines.append(line)

    # Caso en que la sección de control sea la última del archivo
    if in_control_section:
        for key, value in updates.items():
            if key not in updated_keys:
                formatted_value = formatear_float_toml(value, precision)
                new_lines.append(f"{key} = {formatted_value}\n")
                updated_keys.add(key)

    with open(file_path, 'w', encoding='utf-8') as f:
        f.writelines(new_lines)

def main():
    """Función principal CLI."""
    parser = argparse.ArgumentParser(description="Calcula coeficientes de control para RocketPy.")
    parser.add_argument("--toml", type=str, default="data/rockets/leon_2.toml", help="Ruta al archivo TOML del cohete.")
    parser.add_argument("--mach", type=float, default=0.0, help="Número de Mach de diseño (0.0 a <1.0).")
    parser.add_argument("--oswald-efficiency", type=float, default=0.7, help="Factor de eficiencia de Oswald (e).")
    parser.add_argument("--dry-run", action="store_true", help="Imprime los resultados sin modificar el archivo.")
    parser.add_argument("--precision", type=int, default=12, help="Precisión decimal para el redondeo.")

    args = parser.parse_args()

    # Validaciones iniciales
    if not (0 <= args.mach < 1):
        parser.error(f"Mach debe estar entre 0 y 1 (excluido). Valor dado: {args.mach}")

    if args.oswald_efficiency <= 0:
        parser.error(f"La eficiencia de Oswald debe ser positiva. Valor dado: {args.oswald_efficiency}")

    if not os.path.exists(args.toml):
        raise SystemExit(f"Error: Archivo no encontrado: {args.toml}")

    # Leer datos para cálculos usando toml estándar (solo para lectura)
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

        # Realizar cálculos
        a_fin = calcular_area_aleta(root_chord, tip_chord, span)
        
        a_ref_teorica = math.pi * (r_t**2)
        a_ref = control.get("reference_area_m2", a_ref_teorica)
        
        if not math.isclose(a_ref, a_ref_teorica, rel_tol=0.01):
            print(f"Advertencia: reference_area_m2 ({a_ref:.6f}) difiere de pi*r^2 ({a_ref_teorica:.6f})")
        
        gamma_c = calcular_midchord_sweep(root_chord, tip_chord, span, sweep_length)
        beta = calcular_beta(args.mach)
        cn_delta_single = calcular_cn_delta_aislada(span, a_ref, a_fin, beta, gamma_c)
        k_tb = calcular_interferencia_ktb(r_t, span)

        cn_delta_total = 2 * cn_delta_single * k_tb
        cy_delta_total = cn_delta_total
        k_drag_induced = calcular_k_drag_inducido(a_fin, span, args.oswald_efficiency)

        # Resumen
        print("\n--- Resumen de Cálculos Científicos ---")
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
            print("\nDry-run: No se han guardado cambios.")
        else:
            actualizar_toml_preservando_formato(args.toml, updates, args.precision)
            print(f"\nÉxito: Archivo {args.toml} actualizado preservando formato.")

    except KeyError as e:
        raise SystemExit(f"Error: Falta la clave {e} en el archivo TOML.")
    except Exception as e:
        raise SystemExit(f"Error inesperado: {e}")

if __name__ == "__main__":
    main()
