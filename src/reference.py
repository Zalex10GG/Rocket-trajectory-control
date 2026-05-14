"""
Trajectory reference management for the rocket simulation.

Handles loading, interpolation, and numerical differentiation of 
reference trajectories from CSV files.
"""

import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

def load_reference_trajectory(path):
    """
    Loads and prepares the reference trajectory from CSV.
    
    Parameters
    ----------
    path : str
        Path to the reference CSV file.
    """
    df = pd.read_csv(path)
    times = df['time_s'].values
    reference = {'time_s': times, 'interpolators': {}}
    
    for col in df.columns:
        if col != 'time_s':
            reference['interpolators'][col] = interp1d(
                times, df[col].values, 
                kind='linear', 
                bounds_error=False,
                fill_value=(df[col].values[0], df[col].values[-1])
            )
    
    if 'z_enu_m' in df.columns:
        reference['peak_z_enu'] = float(df['z_enu_m'].max())
            
    return reference

def sample_reference(reference, time_s):
    """Samples the reference trajectory at a given time."""
    sample = {'time_s': time_s}
    for col, interpolator in reference['interpolators'].items():
        sample[col] = float(interpolator(time_s))
    
    sample['position_enu_m'] = np.array([sample['x_enu_m'], sample['y_enu_m'], sample['z_enu_m']])
    sample['velocity_enu_m_s'] = np.array([sample['vx_enu_m_s'], sample['vy_enu_m_s'], sample['vz_enu_m_s']])
    return sample

def compute_reference_acceleration(reference, time_s, dt=0.01):
    """Numerically computes reference acceleration from velocity."""
    t_start, t_end = reference['time_s'][0], reference['time_s'][-1]

    if time_s - dt >= t_start and time_s + dt <= t_end:
        v0 = sample_reference(reference, time_s - dt)['velocity_enu_m_s']
        v1 = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
        return (v1 - v0) / (2.0 * dt)
    elif time_s + dt <= t_end:
        v0 = sample_reference(reference, time_s)['velocity_enu_m_s']
        v1 = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
        return (v1 - v0) / dt
    elif time_s - dt >= t_start:
        v0 = sample_reference(reference, time_s - dt)['velocity_enu_m_s']
        v1 = sample_reference(reference, time_s)['velocity_enu_m_s']
        return (v1 - v0) / dt
    return np.zeros(3)
