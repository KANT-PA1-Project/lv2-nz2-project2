"""Rigid pose geometry bound only; not a sensor uncertainty certificate."""
import math

def rigid_surface_displacement(translation_m, rotation_rad, radius_m):
    """Bound any point within radius of the pose origin under a rigid error."""
    for value in (translation_m,rotation_rad,radius_m):
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
            raise ValueError('finite_nonnegative_pose_bound_required')
    if rotation_rad>math.pi:
        raise ValueError('principal_rotation_bound_required')
    return translation_m+2*radius_m*math.sin(rotation_rad/2)

def remaining_external_clearance(nominal_m, translation_m, rotation_rad, radius_m):
    """Subtract object-pose displacement; other error sources remain external."""
    if isinstance(nominal_m,bool) or not isinstance(nominal_m,(int,float)) or not math.isfinite(nominal_m) or nominal_m<0:
        raise ValueError('nominal_clearance_required')
    return nominal_m-rigid_surface_displacement(translation_m,rotation_rad,radius_m)
