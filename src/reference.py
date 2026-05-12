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
            # Use 'constant' fill_value for out of bounds instead of 'extrapolate'
            # to avoid diverging references. For z_enu_m specifically, we might 
            # want to stay at the last value.
            reference['interpolators'][col] = interp1d(
                times, df[col].values, 
                kind='linear', 
                bounds_error=False,
                fill_value=(df[col].values[0], df[col].values[-1])
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

    Uses central differences when possible, and forward/backward differences
    near the reference time boundaries to avoid flat-extrapolation artifacts.

    Parameters
    ----------
    reference : dict
        Loaded reference trajectory (see ``load_reference_trajectory``).
    time_s : float
        Current simulation time (s).
    dt : float
        Finite-difference step (s).

    Returns
    -------
    numpy.ndarray
        Reference acceleration vector [ax, ay, az] in m/s².
    """
    t_start = reference['time_s'][0]
    t_end = reference['time_s'][-1]

    if time_s - dt >= t_start and time_s + dt <= t_end:
        # Central difference (most accurate, O(dt²) error)
        v_before = sample_reference(reference, time_s - dt)['velocity_enu_m_s']
        v_after = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
        return (v_after - v_before) / (2.0 * dt)
    elif time_s + dt <= t_end:
        # Forward difference near start boundary
        v0 = sample_reference(reference, time_s)['velocity_enu_m_s']
        v1 = sample_reference(reference, time_s + dt)['velocity_enu_m_s']
        return (v1 - v0) / dt
    elif time_s - dt >= t_start:
        # Backward difference near end boundary
        v0 = sample_reference(reference, time_s - dt)['velocity_enu_m_s']
        v1 = sample_reference(reference, time_s)['velocity_enu_m_s']
        return (v1 - v0) / dt
    else:
        # Fallback: at a single point or window smaller than dt
        return np.zeros(3)
