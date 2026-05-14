"""
FinAdapter: bridges controller deflection commands to RocketPy GenericSurface
aerodynamic coefficients.

Architecture note (critical):
    In RocketPy 1.12.1, ``Function`` objects wrapping callables keep those
    callables live and evaluate them through ``source(*args)``. The coefficient
    functions therefore read the latest controller deltas at evaluation time.
    A module-level controller-state reference is still used so any copied
    adapter/function path reads the same mutable controller state.
"""

import numpy as np
from rocketpy import Function

# Module-level mutable singleton for controller state.
# Both the controller callback (controllers.fin_controller) and the
# FinAdapter coefficient functions MUST read/write through this reference.
_CONTROLLER_STATE = {}


def set_controller_state_ref(controller_state: dict) -> None:
    """
    Register the controller state dict as the module-level singleton.

    Call this ONCE from ``build_rocket`` (or ``main.py``) before the
    simulation starts.  After this call, all FinAdapter instances and
    coefficient Functions will read from ``controller_state``.

    Parameters
    ----------
    controller_state : dict
        The mutable controller state dictionary created by
        ``controllers.build_controller``.
    """
    global _CONTROLLER_STATE
    # Simply point the module global at the caller's dict object.
    # This is the cleanest approach: _CONTROLLER_STATE IS controller_state.
    # Any in-place mutations by the controller callback (e.g. setting
    # ``current_deltas``) are immediately visible to FinAdapter coefficient
    # functions, even after RocketPy deep-copies the rocket.
    _CONTROLLER_STATE = controller_state


def get_controller_state() -> dict:
    """
    Returns the module-level controller state singleton.

    Returns
    -------
    dict
        The live controller state dictionary.
    """
    return _CONTROLLER_STATE


class FinAdapter:
    """
    Stateful adapter connecting controller fin deflections to GenericSurface
    aerodynamic coefficients.

    The adapter reads ``current_deltas`` from the module-level controller
    state singleton (``_CONTROLLER_STATE``), ensuring that copies of the
    FinAdapter created by RocketPy internally always see the live state.

    Parameters
    ----------
    controller_state : dict
        Mutable controller state dictionary.
    actuation_params : dict
        Actuation parameters from the rocket TOML
        (``case_data["rocket_params"]["control_actuation"]``).
    """

    def __init__(self, controller_state, actuation_params):
        self.controller_state = controller_state or {}
        self.params = actuation_params

        # Aerodynamic derivatives
        self.cN_delta = self.params.get("cN_delta_per_rad", 0.0)
        self.cy_delta = self.params.get("cy_delta_per_rad", 0.0)
        self.cl_delta = self.params.get("cl_delta_per_rad", 0.0)
        self.k_drag_induced = self.params.get("k_drag_induced", 0.0)

    def get_current_deltas(self):
        """
        Returns the latest fin deflection commands from the controller.

        Reads from the module-level singleton to survive RocketPy's internal
        object copying.

        Returns
        -------
        numpy.ndarray
            Array of 4 fin deflections [delta1, delta2, delta3, delta4] in rad.
        """
        return _CONTROLLER_STATE.get("current_deltas", np.zeros(4))

    def cl_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        Lift coefficient (cL) increment from fin deflection.

        Only models the incremental control force.  Passive aerodynamic
        stability is handled by the ``TrapezoidalFins`` object in RocketPy.

        The sign of ``cN_delta`` in the rocket TOML is calibrated against
        RocketPy's aerodynamic frame convention ``[Q, -L, -D]``, so it is
        used directly without negation.

        Note: alpha/beta are NOT used (Fallo 6 — explicit exclusion).
        """
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[0] - deltas[2]) / 2.0
        return self.cN_delta * delta_pitch

    def cq_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Side force coefficient (cQ) increment from fin deflection."""
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[1] - deltas[3]) / 2.0
        return self.cy_delta * delta_yaw

    def cd_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Drag coefficient (cD) including induced drag from control surfaces."""
        cL = self.cl_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)
        cQ = self.cq_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)
        return self.k_drag_induced * (cL**2 + cQ**2)

    def cm_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Pitch moment coefficient (cm). Returns 0 (normal force model)."""
        return 0.0

    def cn_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Yaw moment coefficient (cn). Returns 0 (normal force model)."""
        return 0.0

    def cl_roll_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Roll moment coefficient (cl) from differential fin deflection."""
        deltas = self.get_current_deltas()
        delta_roll = np.mean(deltas)
        return self.cl_delta * delta_roll

    def get_coefficients_dict(self):
        """
        Returns a dict of RocketPy ``Function`` objects for ``GenericSurface``.

        Each coefficient function reads from the module-level controller state
        singleton at evaluation time, not from the FinAdapter instance.
        """
        inputs = ["alpha", "beta", "mach", "reynolds",
                   "pitch_rate", "yaw_rate", "roll_rate"]
        return {
            "cL": Function(self.cl_coeff, inputs, ["cL"]),
            "cQ": Function(self.cq_coeff, inputs, ["cQ"]),
            "cD": Function(self.cd_coeff, inputs, ["cD"]),
            "cm": Function(self.cm_coeff, inputs, ["cm"]),
            "cn": Function(self.cn_coeff, inputs, ["cn"]),
            "cl": Function(self.cl_roll_coeff, inputs, ["cl"]),
        }
