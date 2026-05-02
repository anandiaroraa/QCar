import numpy as np

#replacement of utils.angle import angle_mod
def wrap_angle(angle, 
               zero_to_2pi=False, 
               degrees=False):
    """
    Wrap angle(s) into a fixed range.

    Params

    angle : float or array-like
        Input angle(s)
    zero_to_2pi : bool
        If True  -> range [0, 2π)
        If False -> range [-π, π)
    degrees : bool
        If True, input/output are in degrees
        If False, input/output are in radians

    Returns
    
    Wrapped angle (same type as input)
    """

    # Convert to numpy array
    is_scalar = np.isscalar(angle) #checking if i/p is scalar (on number/list/array)
    angle = np.asarray(angle) #i/p to array conversion

    # Convert degrees → radians if needed
    if degrees:
        angle = np.deg2rad(angle)

    # Wrap angle
    if zero_to_2pi:
        wrapped = angle % (2 * np.pi) #taking angle and converting into one full circl range
    else:
        wrapped = (angle + np.pi) % (2 * np.pi) - np.pi #pi-using 10 instead of 350 and 10 instead of 170

    # Convert back to degrees if needed
    if degrees:
        wrapped = np.rad2deg(wrapped)

    # Return same type as input
    if is_scalar:
        return float(wrapped) #one angle i/p- one float returned
    return wrapped

def quat2euler(quat):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    
    Parameters:
    quat : list or array-like
        Quaternion in the form [x, y, z, w]
    
    Returns:
    tuple
        Euler angles (roll, pitch, yaw) in radians
    """
    from scipy.spatial.transform import Rotation as R
    
    r = R.from_quat(quat) #rotation object created from quaternion
    return r.as_euler('xyz', degrees=False) #converted quats into euler angles

def rot_mat_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)


    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


def angle_mod(x, zero_2_2pi=False, degree=False):
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle