import numpy as np
import pandas as pd
from scipy.interpolate import interp1d

def load_reference_trajectory(path):
    """
    Loads and prepares the reference trajectory from CSV.
    """
    df = pd.read_csv(path)
    
    # Create interpolators for each column
    times = df['time_s'].values
    reference = {
        'time_s': times,
        'interpolators': {}
    }
    
    for col in df.columns:
        if col != 'time_s':
            reference['interpolators'][col] = interp1d(
                times, df[col].values, 
                kind='linear', 
                fill_value='extrapolate'
            )
    
    # Add a separate peak finder for the reference
    if 'z_enu_m' in df.columns:
        reference['peak_z_enu'] = float(df['z_enu_m'].max())
    else:
        reference['peak_z_enu'] = 0.0
            
    return reference

def sample_reference(reference, time_s):
    """
    Samples the reference trajectory at a given time.
    """
    sample = {'time_s': time_s}
    for col, interpolator in reference['interpolators'].items():
        sample[col] = float(interpolator(time_s))
    
    # Pack into vectors for convenience
    sample['position_enu_m'] = np.array([sample['x_enu_m'], sample['y_enu_m'], sample['z_enu_m']])
    sample['velocity_enu_m_s'] = np.array([sample['vx_enu_m_s'], sample['vy_enu_m_s'], sample['vz_enu_m_s']])
    
    return sample

def compute_reference_acceleration(reference, time_s, dt=0.01):
    """
    Numerically computes reference acceleration from velocity.
    """
    v1 = sample_reference(reference, time_s)['velocity_enu_m_s']
    v2 = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
    return (v2 - v1) / dt
