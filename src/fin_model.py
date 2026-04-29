import numpy as np
from rocketpy import Function

class FinAdapter:
    def __init__(self, controller_state, actuation_params):
        """
        Stateful adapter that connects the controller deltas to the GenericSurface coefficients.
        
        actuation_params: dict from rocket TOML (case_data["rocket_params"]["control_actuation"])
        """
        self.controller_state = controller_state
        self.params = actuation_params
        
        # Pull coefficients from params
        self.cN_delta = self.params.get("cN_delta_per_rad", 0.0)
        self.cm_delta = self.params.get("cm_delta_per_rad", 0.0)
        self.cy_delta = self.params.get("cy_delta_per_rad", 0.0)
        self.cn_delta = self.params.get("cn_moment_delta_per_rad", 0.0)
        self.cl_delta = self.params.get("cl_delta_per_rad", 0.0)
        
    def get_current_deltas(self):
        """Helper to get the latest deltas calculated by the controller_function."""
        # controller_state["current_deltas"] will be updated by the controller_function
        return self.controller_state.get("current_deltas", np.zeros(4))

    def cl_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """
        Lift coefficient (cL) from fin deflection.
        RocketPy GenericSurface expects cL, cQ, cD in aerodynamic frame.
        For a + configuration (Leon 2):
        delta_pitch = (d2 - d4)/2
        delta_yaw = (d1 - d3)/2
        
        cL corresponds to -Cy in RocketPy body frame? No, GenericSurface uses:
        R1, R2, R3 = rotation_matrix @ Vector([side, -lift, -drag])
        where side=cQ, lift=cL, drag=cD.
        
        In body frame (Leon 2):
        Normal force (Body Y) ~ cN_delta * delta_pitch
        Side force (Body X) ~ cy_delta * delta_yaw
        
        GenericSurface converts (cQ, -cL, -cD) from aero frame to body frame.
        At low alpha/beta, cL is ~ Normal force, cQ is ~ Side force.
        """
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[1] - deltas[3]) / 2.0
        # cL is lift, which in RocketPy aero frame points 'up' (against gravity if vertical).
        # Normal force in Body Y is cN_delta * delta_pitch.
        # At zero alpha/beta, R2 = -lift = -cL. So cL = -NormalForce.
        return -self.cN_delta * delta_pitch

    def cq_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Side force coefficient (cQ)."""
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[0] - deltas[2]) / 2.0
        # R1 = side = cQ. Normal force in Body X is cy_delta * delta_yaw.
        return self.cy_delta * delta_yaw

    def cd_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Drag coefficient (cD)."""
        # Induced drag can be added here if needed: k * (cL^2 + cQ^2)
        return 0.0

    def cm_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Pitch moment coefficient (cm)."""
        deltas = self.get_current_deltas()
        delta_pitch = (deltas[1] - deltas[3]) / 2.0
        return self.cm_delta * delta_pitch

    def cn_coeff(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        """Yaw moment coefficient (cn)."""
        deltas = self.get_current_deltas()
        delta_yaw = (deltas[0] - deltas[2]) / 2.0
        return self.cn_delta * delta_yaw

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
