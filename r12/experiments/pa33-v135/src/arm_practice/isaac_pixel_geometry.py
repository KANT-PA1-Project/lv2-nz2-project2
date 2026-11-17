"""Opt-in index-coordinate intrinsics for the validated Isaac render setup.

640x480, aperture20.955mm, focal13.096875mm, no lens distortion.
Replicator samples pixel centers at (u+.5,v+.5) in the render-plane convention.
OpenCV/ROS integer-index coordinates therefore use principal point319.5/239.5.
This changes neither the lens nor optical axes (x right,y down,z forward).
Only distance_to_image_plane depths in meters are valid for XYZ=ray*depth;
Euclidean distance_to_camera must not be substituted. This is not hardware
calibration or an uncertainty/free-space/clearance certificate.
"""
import os
import json
import numpy as np


def convention():
    value = os.getenv('R12_ISAAC_PIXEL_CENTER_V1', '0')
    if value not in ('0', '1'):
        raise ValueError('R12_ISAAC_PIXEL_CENTER_V1 must be 0 or 1')
    return 'isaac_pixel_center_v1' if value == '1' else 'legacy_integer_v0'


def intrinsics():
    shift = .5 if convention() == 'isaac_pixel_center_v1' else 0.
    return (400., 400., 320.-shift, 240.-shift)


def camera_matrix():
    fx, fy, cx, cy = intrinsics()
    return np.array([[fx, 0., cx], [0., fy, cy], [0., 0., 1.]])


def projection_matrix(*, right=False):
    fx, fy, cx, cy = intrinsics()
    # Existing nominal50mm baseline. Geometry correction does not re-calibrate it.
    return np.array([[fx, 0., cx, -20. if right else 0.],
                     [0., fy, cy, 0.], [0., 0., 1., 0.]])


def metadata():
    return dict(convention=convention(),resolution=[640,480],
                intrinsics=list(intrinsics()),depth='distance_to_image_plane',
                native_depth_unit='m',ros_depth_unit='mm',
                optical_axes='x_right_y_down_z_forward',
                hardware_calibrated=False,uncertainty_certified=False)


def archive_geometry_fields():
    """Plain Unicode scalar survives allow_pickle=False and shared-array transport."""
    return {'pixel_geometry_json': np.array(json.dumps(metadata(), sort_keys=True,
        separators=(',', ':'), allow_nan=False))}


def require_archive_geometry(data):
    """Match capture and consumer geometry before publication/inference/clearance.

    Legacy mode accepts archives without the field for historical playback.
    Opt-in mode requires it. A present contract must match in either mode;
    an explicitly modern archive must never silently enter a legacy consumer.
    This is a software convention check, not physical calibration certification.
    """
    if 'pixel_geometry_json' not in data:
        if convention() == 'isaac_pixel_center_v1':
            raise ValueError('pixel_geometry_metadata_missing')
        return None
    raw = np.asarray(data['pixel_geometry_json'])
    if raw.shape != () or raw.dtype.kind not in ('U', 'S'):
        raise ValueError('pixel_geometry_metadata_not_text_scalar')
    encoded = raw.item()
    if isinstance(encoded, bytes):
        encoded = encoded.decode('utf-8')
    try:
        actual = json.loads(encoded)
    except (ValueError, UnicodeError) as error:
        raise ValueError('pixel_geometry_metadata_invalid_json') from error
    expected = metadata()
    if not isinstance(actual, dict):
        raise ValueError('pixel_geometry_metadata_not_object')
    # Canonical comparison rejects booleans in numeric fields and NaN values.
    for key, value in expected.items():
        try:
            equal = json.dumps(actual[key], sort_keys=True, allow_nan=False) == json.dumps(value, sort_keys=True, allow_nan=False)
        except (KeyError, TypeError, ValueError):
            equal = False
        if not equal:
            raise ValueError('pixel_geometry_metadata_mismatch:' + key)
    return actual
