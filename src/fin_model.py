import numpy as np
from rocketpy import Function

class FinAdapter:
    def __init__(self, controller_state, actuation_params):
        """
        Stateful adapter that connects the controller deltas to the GenericSurface coefficients.
        
        actuation_params: dict from rocket TOML (case_data["rocket_params"]["control_actuation"])
        """
        self.controller_state = controller_state or {}
        self.params = actuation_params
        
        # Pull coefficients from params
        self.cN_delta = self.params.get("cN_delta_per_rad", 0.0)
        self.cy_delta = self.params.get("cy_delta_per_rad", 0.0)
        self.cl_delta = self.params.get("cl_delta_per_rad", 0.0)
        self.k_drag_induced = self.params.get("k_drag_induced", 0.0)
        
        # Passive stability term
        # If the fins are controlled, we need to include their passive lift
        # when they are at an angle of attack (alpha/beta) but with delta=0.
        self.clalpha_fins = self.params.get("clalpha_fins", 0.0)
        
    def get_current_deltas(self):
        """Helper to get the latest deltas calculated by the controller_function."""
        # controller_state["current_deltas"] will be updated by the controller_function
        return self.controller_state.get("current_deltas", np.zeros(4))

    def cl_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        Lift coefficient (cL) from fin deflection.
        """
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[1] - deltas[3]) / 2.0
        
        # We only model the INCREMENTAL control force here.
        # Passive stability is handled by the TrapezoidalFins object in RocketPy.
        return self.cN_delta * delta_pitch

    def cq_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Side force coefficient (cQ)."""
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[0] - deltas[2]) / 2.0
        
        # We only model the INCREMENTAL control force here.
        return self.cy_delta * delta_yaw

    def cd_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Drag coefficient (cD) including induced drag from control surfaces."""
        cL = self.cl_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)
        cQ = self.cq_coeff(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)
        return self.k_drag_induced * (cL**2 + cQ**2)

    def cm_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Pitch moment coefficient (cm)."""
        # User requested to use normal force and let RocketPy calculate moment.
        # So we return 0 here to avoid double counting.
        return 0.0

    def cn_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Yaw moment coefficient (cn)."""
        # User requested to use normal force and let RocketPy calculate moment.
        return 0.0

    def cl_roll_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Roll moment coefficient (cl)."""
        deltas = self.get_current_deltas()
        delta_roll = np.mean(deltas) # (d1+d2+d3+d4)/4
        return self.cl_delta * delta_roll

    def get_coefficients_dict(self):
        """Returns a dict of RocketPy Functions for GenericSurface."""
        inputs = ["alpha", "beta", "mach", "reynolds", "pitch_rate", "yaw_rate", "roll_rate"]
        return {
            "cL": Function(self.cl_coeff, inputs, ["cL"]),
            "cQ": Function(self.cq_coeff, inputs, ["cQ"]),
            "cD": Function(self.cd_coeff, inputs, ["cD"]),
            "cm": Function(self.cm_coeff, inputs, ["cm"]),
            "cn": Function(self.cn_coeff, inputs, ["cn"]),
            "cl": Function(self.cl_roll_coeff, inputs, ["cl"]),
        }
