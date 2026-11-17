"""Semantic message digests for diagnostic publication/observer comparison."""
import hashlib
import json


def message_evidence(image, info):
    stamp = lambda m: m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
    if image.header.frame_id != info.header.frame_id or stamp(image) != stamp(info):
        raise ValueError('image_info_identity_mismatch')
    if (image.width, image.height) != (info.width, info.height):
        raise ValueError('image_info_size_mismatch')
    if image.encoding != 'rgb8' or image.step != image.width * 3 or len(image.data) != image.step * image.height:
        raise ValueError('invalid_rgb_layout')
    calibration = dict(frame_id=info.header.frame_id, width=info.width, height=info.height,
                       distortion_model=info.distortion_model, d=list(info.d), k=list(info.k),
                       r=list(info.r), p=list(info.p), binning_x=info.binning_x, binning_y=info.binning_y,
                       roi={k:getattr(info.roi,k) for k in ('x_offset','y_offset','height','width','do_rectify')})
    encoded=json.dumps(calibration,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    return dict(frame_id=image.header.frame_id,stamp_ns=stamp(image),encoding=image.encoding,
                is_bigendian=image.is_bigendian,step=image.step,
                image_sha256=hashlib.sha256(bytes(image.data)).hexdigest(),
                calibration_sha256=hashlib.sha256(encoded).hexdigest(),
                calibration=calibration,pose_join_verified=False,execution_authorized=False)
