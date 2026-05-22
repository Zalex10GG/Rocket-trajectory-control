"""
Pitch and Roll identification and Ziegler-Nichols tuning tool.

Runs open-loop pitch and roll pulse/step response simulations at max-Q,
identifies plant models, and estimates PID gains via the
Ziegler-Nichols reaction-curve method.

For pitch: a second-order model is fit to attitude and rate simultaneously.
For roll:  a first-order model is fit to roll rate response.

Results are saved in tools/results/.  ZN gains can be copied into config.py.

Usage:
    uv run py tools/tunning.py
"""

import os
import json
from datetime import datetime
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy.signal import savgol_filter, freqs

# ---------------------------------------------------------------------------
# Tuning Parameters (Adjust here)
# ---------------------------------------------------------------------------
STEP_DEG = 2.0
ROLL_STEP_DEG = 2.0
INPUT_TYPE = "pulse"  # "pulse" or "step"
PULSE_DURATION_S = 0.3
WINDOW_S = 2.0
FIT_WINDOW_S = 2.0

# ---------------------------------------------------------------------------
# Project imports
# ---------------------------------------------------------------------------
import config as cfg
import initial_data as init
import src.controllers as controllers
import src.environment_builder as env_builder
import src.reference as reference_mod
import src.rocket_builder as rocket_builder
from src.constants import CONTROL_SURFACE_NAME
from rocketpy import Flight
from rocketpy.control.controller import _Controller
from src.utils import quaternion_conjugate, quaternion_multiply

def load_nominal_case():
    config = cfg.load_config()
    case_data = init.load_initial_case_data(config)
    reference = reference_mod.load_reference_trajectory(config.reference_path)
    controller = controllers.build_controller(config)
    environment = env_builder.build_environment(case_data, config)
    rocket, components = rocket_builder.build_rocket(case_data, config, controller)
    return config, case_data, reference, controller, environment, rocket, components

def find_reference_max_q(reference, environment, config):
    times = reference["time_s"]
    elev = config.elevation_asl_m
    best = {"qbar_max_pa": -1.0}
    for i, t in enumerate(times):
        z_enu = float(reference["interpolators"]["z_enu_m"](t))
        vx = float(reference["interpolators"]["vx_enu_m_s"](t))
        vy = float(reference["interpolators"]["vy_enu_m_s"](t))
        vz = float(reference["interpolators"]["vz_enu_m_s"](t))
        z_asl = elev + z_enu
        try:
            rho = float(environment.density(z_asl))
        except Exception:
            rho = 1.225
        try:
            w_e = float(environment.wind_velocity_x(z_asl))
            w_n = float(environment.wind_velocity_y(z_asl))
        except Exception:
            w_e, w_n = 0.0, 0.0
        v_air = np.array([vx - w_e, vy - w_n, vz])
        speed = float(np.linalg.norm(v_air))
        qbar = 0.5 * rho * speed ** 2
        if qbar > best["qbar_max_pa"]:
            best = {"t_max_q_s": float(t), "qbar_max_pa": qbar, "z_enu_m": z_enu, "speed_m_s": speed, "idx": i}
    return best


# ---------------------------------------------------------------------------
# Pitch identification
# ---------------------------------------------------------------------------

def simulate_pitch_input(rocket, environment, config, step_rad, t_max_q):
    """
    Runs an open-loop pitch pulse simulation at max-Q.

    The pitch pulse uses delta pattern [step, 0, -step, 0] which,
    given the mixer mix(p,y,r)=[p+r, y+r, -p+r, -y+r], corresponds
    to pure pitch (p=step, y=0, r=0).
    """
    controller = controllers.build_controller(config)
    from src.fin_model import set_controller_state_ref
    set_controller_state_ref(controller)

    def open_loop_callback(t, sampling_rate, state, state_history, observed_vars, interactive_objs, sensors, env):
        pos = np.array(state[0:3])
        vel = np.array(state[3:6])
        z_asl = pos[2]
        try:
            rho = float(env.density(z_asl))
        except Exception:
            rho = 1.225
        try:
            w_e, w_n = float(env.wind_velocity_x(z_asl)), float(env.wind_velocity_y(z_asl))
        except Exception:
            w_e, w_n = 0.0, 0.0
        wind_enu = np.array([w_e, w_n, 0.0])
        vel_air = vel - wind_enu
        airspeed = float(np.linalg.norm(vel_air))
        q_dynamic = 0.5 * rho * airspeed ** 2
        controller["last_q_dynamic"] = q_dynamic
        controller["last_airspeed"] = airspeed
        t_float = float(t)
        input_active = t_float >= t_max_q
        if INPUT_TYPE == "pulse":
            input_active = input_active and t_float < t_max_q + PULSE_DURATION_S
        # Pitch: [step, 0, -step, 0]  (p=step, y=0, r=0 in mixer convention)
        deltas = np.array([step_rad, 0.0, -step_rad, 0.0]) if input_active else np.zeros(4)
        controller["current_deltas"] = deltas
        controller["deltas_history"][float(t)] = deltas
        return None

    rocket._controllers = []
    control_surf = next((s.component for s in rocket.aerodynamic_surfaces if hasattr(s.component, "name") and s.component.name == CONTROL_SURFACE_NAME), None)
    ctrl_obj = _Controller(interactive_objects=[control_surf] if control_surf else [], controller_function=open_loop_callback, sampling_rate=1.0 / config.control_dt_s, name="Open-Loop Pitch Pulse")
    rocket._add_controllers(ctrl_obj)

    flight = Flight(rocket=rocket, environment=environment, rail_length=config.rail_length_m, inclination=config.inclination_deg, heading=config.heading_deg, terminate_on_apogee=True, max_time=config.max_time_s, time_overshoot=False, verbose=False)
    _ = flight.apogee_time
    sol = np.array(flight.solution)
    launch_pos_enu = sol[0, 1:4]
    deltas_dict = controller["deltas_history"]
    ctrl_times_sorted = np.array(sorted(deltas_dict.keys()))
    ctrl_deltas_sorted = np.array([deltas_dict[k] for k in ctrl_times_sorted])

    history = []
    for i, t in enumerate(sol[:, 0]):
        state_vec = sol[i, 1:]
        idx = int(np.searchsorted(ctrl_times_sorted, t, side="right"))
        deltas = ctrl_deltas_sorted[idx - 1] if idx > 0 else np.zeros(4)
        history.append({
            "time_s": float(t), "position_enu_m": state_vec[0:3] - launch_pos_enu, "velocity_enu_m_s": state_vec[3:6],
            "attitude_quaternion": state_vec[6:10], "body_rates_rad_s": state_vec[10:13], "deltas": deltas,
        })
    return history, controller


# ---------------------------------------------------------------------------
# Roll identification
# ---------------------------------------------------------------------------

def simulate_roll_input(rocket, environment, config, step_rad, t_max_q):
    """
    Runs an open-loop roll pulse simulation at max-Q.

    The roll pulse uses delta pattern [step, step, step, step] which,
    given the mixer mix(p,y,r)=[p+r, y+r, -p+r, -y+r], corresponds
    to pure roll (p=0, y=0, r=step).  All four fins deflect equally.
    """
    controller = controllers.build_controller(config)
    from src.fin_model import set_controller_state_ref
    set_controller_state_ref(controller)

    def open_loop_callback(t, sampling_rate, state, state_history, observed_vars, interactive_objs, sensors, env):
        pos = np.array(state[0:3])
        vel = np.array(state[3:6])
        z_asl = pos[2]
        try:
            rho = float(env.density(z_asl))
        except Exception:
            rho = 1.225
        try:
            w_e, w_n = float(env.wind_velocity_x(z_asl)), float(env.wind_velocity_y(z_asl))
        except Exception:
            w_e, w_n = 0.0, 0.0
        wind_enu = np.array([w_e, w_n, 0.0])
        vel_air = vel - wind_enu
        airspeed = float(np.linalg.norm(vel_air))
        q_dynamic = 0.5 * rho * airspeed ** 2
        controller["last_q_dynamic"] = q_dynamic
        controller["last_airspeed"] = airspeed
        t_float = float(t)
        input_active = t_float >= t_max_q
        if INPUT_TYPE == "pulse":
            input_active = input_active and t_float < t_max_q + PULSE_DURATION_S
        # Roll: [step, step, step, step] (all fins same direction)
        deltas = np.array([step_rad, step_rad, step_rad, step_rad]) if input_active else np.zeros(4)
        controller["current_deltas"] = deltas
        controller["deltas_history"][float(t)] = deltas
        return None

    rocket._controllers = []
    control_surf = next((s.component for s in rocket.aerodynamic_surfaces if hasattr(s.component, "name") and s.component.name == CONTROL_SURFACE_NAME), None)
    ctrl_obj = _Controller(interactive_objects=[control_surf] if control_surf else [], controller_function=open_loop_callback, sampling_rate=1.0 / config.control_dt_s, name="Open-Loop Roll Pulse")
    rocket._add_controllers(ctrl_obj)

    flight = Flight(rocket=rocket, environment=environment, rail_length=config.rail_length_m, inclination=config.inclination_deg, heading=config.heading_deg, terminate_on_apogee=True, max_time=config.max_time_s, time_overshoot=False, verbose=False)
    _ = flight.apogee_time
    sol = np.array(flight.solution)
    launch_pos_enu = sol[0, 1:4]
    deltas_dict = controller["deltas_history"]
    ctrl_times_sorted = np.array(sorted(deltas_dict.keys()))
    ctrl_deltas_sorted = np.array([deltas_dict[k] for k in ctrl_times_sorted])

    history = []
    for i, t in enumerate(sol[:, 0]):
        state_vec = sol[i, 1:]
        idx = int(np.searchsorted(ctrl_times_sorted, t, side="right"))
        deltas = ctrl_deltas_sorted[idx - 1] if idx > 0 else np.zeros(4)
        history.append({
            "time_s": float(t), "position_enu_m": state_vec[0:3] - launch_pos_enu, "velocity_enu_m_s": state_vec[3:6],
            "attitude_quaternion": state_vec[6:10], "body_rates_rad_s": state_vec[10:13], "deltas": deltas,
        })
    return history, controller


# ---------------------------------------------------------------------------
# Signal extraction
# ---------------------------------------------------------------------------

def extract_pitch_signals(history, t_step_applied_s, environment, config):
    """Extracts pitch attitude and rate in the WINDOW_S after the step."""
    times = np.array([h["time_s"] for h in history])
    baseline_idx = np.where(times < t_step_applied_s - 1e-9)[0][-1] if np.any(times < t_step_applied_s - 1e-9) else 0
    q_baseline = history[baseline_idx]["attitude_quaternion"].copy()
    rows = []
    for h in history:
        t = h["time_s"]
        if t < t_step_applied_s - 0.5 or t > t_step_applied_s + WINDOW_S: continue
        q_rel = quaternion_multiply(h["attitude_quaternion"], quaternion_conjugate(q_baseline))
        z_asl = config.elevation_asl_m + h["position_enu_m"][2]
        try:
            rho = float(environment.density(z_asl))
            w_e, w_n = float(environment.wind_velocity_x(z_asl)), float(environment.wind_velocity_y(z_asl))
        except:
            rho, w_e, w_n = 1.225, 0.0, 0.0
        v_air = h["velocity_enu_m_s"] - np.array([w_e, w_n, 0.0])
        rows.append({
            "time_s": t, "time_since_step_s": t - t_step_applied_s, "qbar_pa": 0.5 * rho * np.linalg.norm(v_air)**2,
            "delta_pitch_deg": np.rad2deg((h["deltas"][0] - h["deltas"][2]) / 2.0),
            "theta_pitch_rad": 2.0 * q_rel[1], "theta_pitch_deg": np.rad2deg(2.0 * q_rel[1]),
            "pitch_rate_rad_s": h["body_rates_rad_s"][0], "pitch_rate_deg_s": np.rad2deg(h["body_rates_rad_s"][0])
        })
    return pd.DataFrame(rows)


def extract_roll_signals(history, t_step_applied_s, environment, config):
    """Extracts roll rate in the WINDOW_S after the step."""
    times = np.array([h["time_s"] for h in history])
    baseline_idx = np.where(times < t_step_applied_s - 1e-9)[0][-1] if np.any(times < t_step_applied_s - 1e-9) else 0
    q_baseline = history[baseline_idx]["attitude_quaternion"].copy()
    rows = []
    for h in history:
        t = h["time_s"]
        if t < t_step_applied_s - 0.5 or t > t_step_applied_s + WINDOW_S: continue
        q_rel = quaternion_multiply(h["attitude_quaternion"], quaternion_conjugate(q_baseline))
        # Roll angle from quaternion error
        roll_rad = 2.0 * q_rel[3]  # small-angle approximation
        z_asl = config.elevation_asl_m + h["position_enu_m"][2]
        try:
            rho = float(environment.density(z_asl))
            w_e, w_n = float(environment.wind_velocity_x(z_asl)), float(environment.wind_velocity_y(z_asl))
        except:
            rho, w_e, w_n = 1.225, 0.0, 0.0
        v_air = h["velocity_enu_m_s"] - np.array([w_e, w_n, 0.0])
        rows.append({
            "time_s": t, "time_since_step_s": t - t_step_applied_s, "qbar_pa": 0.5 * rho * np.linalg.norm(v_air)**2,
            "delta_roll_deg": np.rad2deg(np.mean(h["deltas"])),
            "theta_roll_rad": roll_rad, "theta_roll_deg": np.rad2deg(roll_rad),
            "roll_rate_rad_s": h["body_rates_rad_s"][2], "roll_rate_deg_s": np.rad2deg(h["body_rates_rad_s"][2])
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Model fitting: Pitch (2nd order)
# ---------------------------------------------------------------------------

def fit_second_order_model(df, step_rad):
    """
    Fits a second-order system (zeta, wn, gain K) to pitch attitude
    and rate simultaneously.  Handles both step and pulse inputs.
    """
    mask = (df["time_since_step_s"] >= 0.0) & (df["time_since_step_s"] <= FIT_WINDOW_S)
    t_fit, y_fit, q_fit = df.loc[mask, "time_since_step_s"].values, df.loc[mask, "theta_pitch_rad"].values, df.loc[mask, "pitch_rate_rad_s"].values
    y0_fixed = y_fit[0] if len(y_fit) else 0.0
    q_weight = max(np.std(y_fit), 1e-3) / max(np.std(q_fit), 1e-3)

    def step_resp(t, zeta, wn):
        out = np.zeros_like(t)
        act = t >= 0
        ta = t[act]
        if zeta < 1.0:
            wd = wn * np.sqrt(1.0 - zeta**2)
            out[act] = 1.0 - np.exp(-zeta*wn*ta)*(np.cos(wd*ta) + (zeta/np.sqrt(1-zeta**2))*np.sin(wd*ta))
        else:
            wd = wn * np.sqrt(zeta**2 - 1.0)
            s1, s2 = -zeta*wn + wd, -zeta*wn - wd
            if abs(s1-s2) < 1e-12: out[act] = 1.0 - (1.0 + s1*ta)*np.exp(s1*ta)
            else: out[act] = 1.0 - (s2/(s2-s1))*np.exp(s1*ta) - (-s1/(s2-s1))*np.exp(s2*ta)
        return out

    def step_der(t, zeta, wn):
        out = np.zeros_like(t)
        act = t >= 0
        ta = t[act]
        if zeta < 1.0:
            wd = wn * np.sqrt(1.0 - zeta**2)
            out[act] = (wn**2/wd)*np.exp(-zeta*wn*ta)*np.sin(wd*ta)
        else:
            wd = wn * np.sqrt(zeta**2 - 1.0)
            s1, s2 = -zeta*wn + wd, -zeta*wn - wd
            if abs(s1-s2) < 1e-12: out[act] = -(2*s1 + s1**2*ta)*np.exp(s1*ta)
            else: out[act] = -(s2/(s2-s1))*s1*np.exp(s1*ta) - (-s1/(s2-s1))*s2*np.exp(s2*ta)
        return out

    def model_full(t, K, zeta, wn, drift):
        resp = step_resp(t, zeta, wn)
        if INPUT_TYPE == "pulse": resp -= step_resp(t - PULSE_DURATION_S, zeta, wn)
        return y0_fixed + drift*t + K*step_rad*resp

    def model_der(t, K, zeta, wn, drift):
        der = step_der(t, zeta, wn)
        if INPUT_TYPE == "pulse": der -= step_der(t - PULSE_DURATION_S, zeta, wn)
        return drift + K*step_rad*der

    def resid(t, K, zeta, wn, drift):
        return np.concatenate([model_full(t, K, zeta, wn, drift) - y_fit, q_weight * (model_der(t, K, zeta, wn, drift) - q_fit)])

    try:
        popt, _ = curve_fit(resid, t_fit, np.zeros(2*len(t_fit)), p0=[-1.0, 0.2, 20.0, 0.0], bounds=([-20, 0.01, 0.1, -1], [20, 2.0, 100, 1]))
        K, zeta, wn, drift = popt
        full_t = df["time_since_step_s"].values
        return {"K": K, "zeta": zeta, "wn": wn, "drift": drift, "y_model": model_full(full_t, *popt), "q_model": model_der(full_t, *popt), "status": "ok"}
    except:
        return {"status": "failed"}


# ---------------------------------------------------------------------------
# Model fitting: Roll (1st-order on rate)
# ---------------------------------------------------------------------------

def fit_roll_model(df, step_rad):
    """
    Fits a first-order system G(s) = K/(tau*s + 1) to the roll rate
    step response.  Handles both step and pulse inputs.

    Returns the model fit with K (gain), tau (time constant).
    """
    mask = (df["time_since_step_s"] >= 0.0) & (df["time_since_step_s"] <= FIT_WINDOW_S)
    t_fit = df.loc[mask, "time_since_step_s"].values
    y_fit = df.loc[mask, "roll_rate_rad_s"].values  # roll rate, rad/s
    y0_fixed = y_fit[0] if len(y_fit) else 0.0

    def first_order_step(t, K, tau):
        """First-order step response: K * (1 - exp(-t/tau))."""
        out = np.zeros_like(t)
        act = t >= 0
        out[act] = K * (1.0 - np.exp(-t[act] / tau))
        return out

    def model(t, K, tau, drift):
        resp = first_order_step(t, K, tau)
        if INPUT_TYPE == "pulse":
            resp -= first_order_step(t - PULSE_DURATION_S, K, tau)
        return y0_fixed + drift * t + step_rad * resp

    def resid(t, K, tau, drift):
        return model(t, K, tau, drift) - y_fit

    try:
        # Initial guess: K ~ steady-state rate / step, tau ~ 0.05-0.2s
        K_guess = (y_fit[-1] - y_fit[0]) / max(step_rad, 1e-6) if len(y_fit) > 1 else 1.0
        popt, _ = curve_fit(resid, t_fit, np.zeros(len(t_fit)),
                            p0=[K_guess, 0.1, 0.0],
                            bounds=([-100, 0.005, -10], [100, 5.0, 10]))
        K, tau, drift = popt
        full_t = df["time_since_step_s"].values
        return {"K": K, "tau": tau, "drift": drift,
                "y_model": model(full_t, K, tau, drift),
                "status": "ok"}
    except:
        return {"status": "failed"}


# ---------------------------------------------------------------------------
# Ziegler-Nichols reaction-curve (pitch)
# ---------------------------------------------------------------------------

def compute_zn(df, fit):
    """Computes ZN reaction-curve PID gains for pitch."""
    if fit["status"] != "ok": return {"confidence": "low"}
    t, theta = df.loc[df["time_since_step_s"] >= 0, "time_since_step_s"].values, df.loc[df["time_since_step_s"] >= 0, "theta_pitch_rad"].values
    y_smooth = savgol_filter(theta, min(11, len(theta) if len(theta)%2 else len(theta)-1), 3) if len(theta) > 5 else theta
    slope = np.diff(y_smooth) / np.diff(t)
    idx = np.argmax(np.abs(slope))
    max_slope, t_at, y_at = slope[idx], t[idx], y_smooth[idx]
    L = t_at + (y_smooth[0] - y_at) / max_slope
    R = abs(max_slope) / (abs(np.deg2rad(STEP_DEG)) * (PULSE_DURATION_S if INPUT_TYPE == "pulse" else 1.0))
    a = R * L
    if a <= 0 or L <= 0: return {"confidence": "invalid"}
    Kp = 1.2 / a
    return {"Kp": Kp, "Ki": Kp / (2*L), "Kd": Kp * (0.5*L), "L": L, "R": R, "confidence": "valid"}


def compute_zn_roll(df, fit):
    """
    Computes ZN reaction-curve PID gains for roll, treating the
    roll rate step response as a first-order self-regulating process.

    If the reaction-curve tangent method fails (L <= 0 or very small),
    falls back to using the identified first-order model (K, tau) with
    a conservative dead-time estimate derived from the simulation timestep.
    """
    if fit["status"] != "ok": return {"confidence": "low"}
    t = df.loc[df["time_since_step_s"] >= 0, "time_since_step_s"].values
    y = df.loc[df["time_since_step_s"] >= 0, "roll_rate_rad_s"].values
    y_smooth = savgol_filter(y, min(11, len(y) if len(y)%2 else len(y)-1), 3) if len(y) > 5 else y

    # Steady-state gain from final value
    step_amplitude = np.deg2rad(ROLL_STEP_DEG) * (PULSE_DURATION_S if INPUT_TYPE == "pulse" else 1.0)

    # Reaction-curve tangent method
    slope = np.diff(y_smooth) / np.diff(t)
    idx = np.argmax(np.abs(slope))
    max_slope = slope[idx]
    t_at, y_at = t[idx], y_smooth[idx]
    L = t_at + (y_smooth[0] - y_at) / max_slope if abs(max_slope) > 1e-9 else 0.0
    R = (y_smooth[-1] - y_smooth[0]) / max(step_amplitude, 1e-6)

    # Tangential method may produce L < 0 for responses with no apparent dead time
    if L <= 0.0 or R <= 0.0:
        # Fallback: use identified model with a practical minimum dead time.
        # The effective dead time cannot be smaller than the control loop
        # period (50 Hz = 0.02 s), which sets a realistic upper bound on Ki.
        K_fit = fit["K"]
        tau_fit = max(fit["tau"], 0.005)
        # Use tau as dominant pole; estimate dead time conservatively
        L = max(tau_fit, 0.02)  # at least one control period
        R = K_fit / tau_fit
        if R <= 0:
            return {"confidence": "invalid"}

    a = R * L
    if a <= 0: return {"confidence": "invalid"}
    Kp = 1.2 / a
    return {"Kp": Kp, "Ki": Kp / (2 * L), "Kd": Kp * (0.5 * L), "L": L, "R": R, "confidence": "valid"}


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def plot_pitch_results(df, fit, zn, out_dir):
    """Generates pitch identification plots."""
    import matplotlib.pyplot as plt

    # 1. Time Response Plot (Theta & Pitch Rate)
    fig1, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    axes[0].plot(df["time_since_step_s"], df["theta_pitch_deg"], label="Measured")
    if fit["status"] == "ok": axes[0].plot(df["time_since_step_s"], np.rad2deg(fit["y_model"]), '--', label="Fit")
    axes[0].set_ylabel("Theta (deg)"); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(df["time_since_step_s"], df["pitch_rate_deg_s"], label="Measured q")
    if fit["status"] == "ok": axes[1].plot(df["time_since_step_s"], np.rad2deg(fit["q_model"]), '--', label="Fit q")
    axes[1].set_ylabel("q (deg/s)"); axes[1].legend(); axes[1].grid(True)
    axes[2].step(df["time_since_step_s"], df["delta_pitch_deg"], where='post')
    axes[2].set_ylabel("Delta (deg)"); axes[2].set_xlabel("Time (s)"); axes[2].grid(True)
    fig1.tight_layout(); fig1.savefig(os.path.join(out_dir, "pitch_identification.png"))
    plt.close(fig1)

    if fit["status"] != "ok":
        return

    # 2. Bode & Nichols
    w = np.logspace(-1, 3, 1000)
    num, den = [fit["K"] * fit["wn"]**2], [1.0, 2.0 * fit["zeta"] * fit["wn"], fit["wn"]**2]
    _, h = freqs(num, den, worN=w)
    mag, phase = 20 * np.log10(np.abs(h)), np.angle(h, deg=True)
    # Bode
    fig2, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax_m.semilogx(w, mag); ax_m.set_ylabel("Magnitude (dB)"); ax_m.grid(True, which="both")
    ax_p.semilogx(w, phase); ax_p.set_ylabel("Phase (deg)"); ax_p.set_xlabel("Frequency (rad/s)"); ax_p.grid(True, which="both")
    fig2.suptitle(f"Bode Plot | zeta={fit['zeta']:.3f}, wn={fit['wn']:.2f} rad/s")
    fig2.savefig(os.path.join(out_dir, "pitch_bode.png"))
    plt.close(fig2)
    # Nichols
    fig3, ax_n = plt.subplots(figsize=(8, 8))
    ax_n.plot(phase, mag, label="Plant P(s)")
    if zn["confidence"] == "valid":
        s = 1j * w; C = zn["Kp"] * (1.0 + 1.0/(2*zn["L"]*s) + (0.5*zn["L"])*s)
        h_loop = C * h; ax_n.plot(np.angle(h_loop, deg=True), 20 * np.log10(np.abs(h_loop)), '--', label="Loop L(s)")
    ax_n.set_xlabel("Phase (deg)"); ax_n.set_ylabel("Magnitude (dB)"); ax_n.grid(True); ax_n.legend()
    ax_n.set_title(f"Nichols Chart | zeta={fit['zeta']:.3f}, wn={fit['wn']:.2f} rad/s")
    fig3.savefig(os.path.join(out_dir, "pitch_nichols.png"))
    plt.close(fig3)

    # 3. Pole Map
    fig4, ax_map = plt.subplots(figsize=(8, 8))
    if fit["zeta"] < 1.0:
        wd = fit["wn"] * np.sqrt(1.0 - fit["zeta"]**2)
        poles = [complex(-fit["zeta"]*fit["wn"], wd), complex(-fit["zeta"]*fit["wn"], -wd)]
    else:
        wd = fit["wn"] * np.sqrt(fit["zeta"]**2 - 1.0)
        poles = [complex(-fit["zeta"]*fit["wn"] + wd, 0), complex(-fit["zeta"]*fit["wn"] - wd, 0)]
    for p in poles: ax_map.plot(p.real, p.imag, 'rx', markersize=14, markeredgewidth=2.5, label=f"Pitch Poles (s = {p.real:.2f} ± {abs(p.imag):.2f}j)" if p.imag >= 0 else "")
    for z in [0.5, 0.707, 0.9]:
        angle = np.pi - np.arccos(z)
        ax_map.plot([0, 100*np.cos(angle)], [0, 100*np.sin(angle)], 'k--', alpha=0.2)
        ax_map.plot([0, 100*np.cos(angle)], [0, -100*np.sin(angle)], 'k--', alpha=0.2)
        ax_map.text(40*np.cos(angle), 40*np.sin(angle), f"ζ = {z}", fontsize=8, alpha=0.5, color='gray', ha='right', va='bottom')
    for r in [10, 20, 30, 40, 50]: 
        ax_map.add_patch(plt.Circle((0,0), r, color='k', fill=False, linestyle=':', alpha=0.2))
        ax_map.text(0.5, r + 0.5, f"ωn = {r} rad/s", fontsize=8, alpha=0.5, color='gray', ha='left')
    ax_map.axhline(0, color='k', lw=1.5); ax_map.axvline(0, color='k', lw=1.5); ax_map.grid(True, alpha=0.3)
    ax_map.set_xlabel("Real (1/s)"); ax_map.set_ylabel("Imag (rad/s)"); ax_map.set_title(f"Pole Map | zeta={fit['zeta']:.3f}, wn={fit['wn']:.2f} rad/s"); ax_map.set_aspect('equal')
    ax_map.legend(loc="upper left")
    max_p = max(fit['wn'] * 1.2, 10); ax_map.set_xlim(-max_p, max_p*0.1); ax_map.set_ylim(-max_p, max_p)
    fig4.savefig(os.path.join(out_dir, "pitch_modes.png"))
    plt.close(fig4)


def plot_roll_results(df, fit, zn, out_dir):
    """Generates roll identification plots."""
    import matplotlib.pyplot as plt

    # 1. Time Response (Roll Rate)
    fig1, axes = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    axes[0].plot(df["time_since_step_s"], df["theta_roll_deg"], label="Measured roll angle")
    axes[0].set_ylabel("Roll angle (deg)"); axes[0].legend(); axes[0].grid(True)
    axes[1].plot(df["time_since_step_s"], df["roll_rate_deg_s"], label="Measured roll rate")
    if fit["status"] == "ok":
        axes[1].plot(df["time_since_step_s"], np.rad2deg(fit["y_model"]), '--', label="Fit")
    axes[1].set_ylabel("Roll rate (deg/s)"); axes[1].legend(); axes[1].grid(True)
    axes[2].step(df["time_since_step_s"], df["delta_roll_deg"], where='post')
    axes[2].set_ylabel("Delta (deg)"); axes[2].set_xlabel("Time (s)"); axes[2].grid(True)
    fig1.tight_layout(); fig1.savefig(os.path.join(out_dir, "roll_identification.png"))
    plt.close(fig1)

    if fit["status"] != "ok":
        return

    # 2. Bode plot for first-order plant
    w = np.logspace(-1, 3, 1000)
    K, tau = fit["K"], fit["tau"]
    _, h = freqs([K], [tau, 1.0], worN=w)
    mag, phase = 20 * np.log10(np.abs(h)), np.angle(h, deg=True)

    fig2, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax_m.semilogx(w, mag); ax_m.set_ylabel("Magnitude (dB)"); ax_m.grid(True, which="both")
    ax_p.semilogx(w, phase); ax_p.set_ylabel("Phase (deg)"); ax_p.set_xlabel("Frequency (rad/s)"); ax_p.grid(True, which="both")
    fig2.suptitle(f"Roll Bode Plot | K={K:.3f}, tau={tau:.4f}s")
    fig2.savefig(os.path.join(out_dir, "roll_bode.png"))
    plt.close(fig2)

    # 3. Roll Pole Map
    fig3, ax_map = plt.subplots(figsize=(8, 8))
    pole = -1.0 / tau
    # Plot Roll Damping Pole
    ax_map.plot(pole, 0.0, 'rx', markersize=14, markeredgewidth=2.5, label=f"Roll Damping Pole (s = {pole:.2f} rad/s)")
    # Also plot the integrator pole at s = 0 if we consider roll angle attitude G(s) = K/[s*(tau*s + 1)]
    ax_map.plot(0.0, 0.0, 'bo', markersize=10, markeredgewidth=2, label="Integrator / Attitude Pole (s = 0)")
    
    ax_map.axhline(0, color='k', lw=1.5)
    ax_map.axvline(0, color='k', lw=1.5)
    ax_map.grid(True, alpha=0.3)
    ax_map.set_xlabel("Real (1/s)")
    ax_map.set_ylabel("Imag (rad/s)")
    ax_map.set_title(f"Roll Dynamics Pole Map | τ = {tau:.5f} s")
    ax_map.legend(loc="upper left")
    
    # Adjust axes limits to display the poles clearly
    xlim_min = pole * 1.2
    xlim_max = abs(pole) * 0.2
    ax_map.set_xlim(xlim_min, xlim_max)
    ax_map.set_ylim(-abs(pole)*0.5, abs(pole)*0.5)
    ax_map.set_aspect('equal', adjustable='box')
    
    fig3.savefig(os.path.join(out_dir, "roll_modes.png"))
    plt.close(fig3)



# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("--- Simplified Rocket Tuning Tool (Pitch + Roll) ---")
    config, case_data, ref, _, env, rocket, _ = load_nominal_case()
    mq = find_reference_max_q(ref, env, config)
    print(f"Max-Q at t={mq['t_max_q_s']:.2f}s, qbar={mq['qbar_max_pa']:.1f}Pa")

    out_dir = os.path.join(os.path.dirname(__file__), "results")
    os.makedirs(out_dir, exist_ok=True)

    results_summary = {"max_q": mq}

    # ---- Pitch Identification ----
    print("\n[1/2] Pitch identification...")
    history_pitch, sim_ctrl_pitch = simulate_pitch_input(rocket, env, config, np.deg2rad(STEP_DEG), mq['t_max_q_s'])
    t_step_pitch = next((t for t, d in sorted(sim_ctrl_pitch["deltas_history"].items()) if abs((d[0]-d[2])/2.0) > 1e-4), mq['t_max_q_s'])
    df_pitch = extract_pitch_signals(history_pitch, t_step_pitch, env, config)
    df_pitch.to_csv(os.path.join(out_dir, "pitch_step_response.csv"), index=False)

    fit_pitch = fit_second_order_model(df_pitch, np.deg2rad(STEP_DEG))
    zn_pitch = compute_zn(df_pitch, fit_pitch)

    if fit_pitch["status"] == "ok":
        print(f"  Pitch plant: K={fit_pitch['K']:.3f}, zeta={fit_pitch['zeta']:.3f}, wn={fit_pitch['wn']:.3f} rad/s")
        if zn_pitch["confidence"] == "valid":
            print(f"  Pitch ZN gains: Kp={zn_pitch['Kp']:.6f}, Ki={zn_pitch['Ki']:.6f}, Kd={zn_pitch['Kd']:.6f}")
        results_summary["pitch"] = {"fit": {k: float(v) if isinstance(v, (np.floating, float, np.integer, int)) else v for k, v in fit_pitch.items() if k != "y_model" and k != "q_model"}, "zn": zn_pitch}
    else:
        print("  Pitch identification FAILED")
        results_summary["pitch"] = {"status": "failed"}

    plot_pitch_results(df_pitch, fit_pitch, zn_pitch, out_dir)

    # ---- Roll Identification ----
    print("\n[2/2] Roll identification...")
    history_roll, sim_ctrl_roll = simulate_roll_input(rocket, env, config, np.deg2rad(ROLL_STEP_DEG), mq['t_max_q_s'])
    t_step_roll = next((t for t, d in sorted(sim_ctrl_roll["deltas_history"].items()) if abs(np.mean(d)) > 1e-4), mq['t_max_q_s'])
    df_roll = extract_roll_signals(history_roll, t_step_roll, env, config)
    df_roll.to_csv(os.path.join(out_dir, "roll_step_response.csv"), index=False)

    fit_roll = fit_roll_model(df_roll, np.deg2rad(ROLL_STEP_DEG))
    zn_roll = compute_zn_roll(df_roll, fit_roll)

    if fit_roll["status"] == "ok":
        print(f"  Roll plant: K={fit_roll['K']:.3f}, tau={fit_roll['tau']:.4f}s")
        if zn_roll["confidence"] == "valid":
            print(f"  Roll ZN gains: Kp={zn_roll['Kp']:.6f}, Ki={zn_roll['Ki']:.6f}, Kd={zn_roll['Kd']:.6f}")
        results_summary["roll"] = {"fit": {k: float(v) if isinstance(v, (np.floating, float, np.integer, int)) else v for k, v in fit_roll.items() if k != "y_model"}, "zn": zn_roll}
    else:
        print("  Roll identification FAILED")
        results_summary["roll"] = {"status": "failed"}

    plot_roll_results(df_roll, fit_roll, zn_roll, out_dir)

    # Export summary JSON
    with open(os.path.join(out_dir, "tuning_summary.json"), "w") as f:
        json.dump(results_summary, f, indent=4, default=str)

    print(f"\nResults saved in {out_dir}")

    # Print copy-paste config snippet
    if zn_pitch.get("confidence") == "valid" and zn_roll.get("confidence") == "valid":
        print("\n--- Config snippet (copy to config.py) ---")
        print(f"    Kp_attitude: float = {zn_pitch['Kp']:.6f}")
        print(f"    Ki_attitude: float = {zn_pitch['Ki']:.6f}")
        print(f"    Kd_attitude: float = {zn_pitch['Kd']:.6f}")
        print(f"    Kp_roll: float = {zn_roll['Kp']:.6f}")
        print(f"    Ki_roll: float = {zn_roll['Ki']:.6f}")
        print(f"    Kd_roll: float = {zn_roll['Kd']:.6f}")


if __name__ == "__main__":
    main()
