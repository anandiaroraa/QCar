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
    is_scalar = np.isscalar(angle)
    angle = np.asarray(angle)

    # Convert degrees → radians if needed
    if degrees:
        angle = np.deg2rad(angle)

    # Wrap angle
    if zero_to_2pi:
        wrapped = angle % (2 * np.pi)
    else:
        wrapped = (angle + np.pi) % (2 * np.pi) - np.pi

    # Convert back to degrees if needed
    if degrees:
        wrapped = np.rad2deg(wrapped)

    # Return same type as input
    if is_scalar:
        return float(wrapped)
    return wrapped