import numpy as np
import pandas as pd
import os

"""
gen_reference.py
Utility script to generate reference trajectories for the rocket control simulation.
Standard output is a CSV file in data/trajectory/ used by src.reference.sample_reference.
"""

def generate_vertical_reference(output_path, max_altitude=1000, duration=20, dt=0.01):
    """
    Generates a simple vertical reference trajectory.
    Reaches max_altitude at half duration with a smooth profile.
    """
    t = np.arange(0, duration + dt, dt)
    
    # Smooth altitude profile: z(t) = H * (1 - cos(pi * t / T)) / 2 for t < T
    # Simplified: linear up and down for V1 is enough, but let's make it slightly better.
    # Actually, a constant velocity up followed by constant velocity down is easier to follow.
    
    z = np.zeros_like(t)
    vz = np.zeros_like(t)
    
    v_const = (2 * max_altitude) / duration
    
    for i, ti in enumerate(t):
        if ti <= duration / 2:
            z[i] = v_const * ti
            vz[i] = v_const
        else:
            z[i] = max_altitude - v_const * (ti - duration / 2)
            vz[i] = -v_const
            
    df = pd.DataFrame({
        'time_s': t,
        'x_enu_m': 0.0,
        'y_enu_m': 0.0,
        'z_enu_m': z,
        'vx_enu_m_s': 0.0,
        'vy_enu_m_s': 0.0,
        'vz_enu_m_s': vz
    })
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    df.to_csv(output_path, index=False)
    print(f"Reference trajectory saved to {output_path}")

if __name__ == "__main__":
    generate_vertical_reference("data/trajectory/vertical.csv")
