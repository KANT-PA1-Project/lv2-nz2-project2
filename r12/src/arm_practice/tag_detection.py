"""Multiscale image-only tag decoding with spatial agreement and conflict rejection."""
import numpy as np
from .mission_markers import marker_spec


def agreed_tags(observations):
    groups = []
    for scale, identity, corners in observations:
        corners = np.asarray(corners, float)
        center = corners.mean(axis=0)
        radius = max(3., np.linalg.norm(corners.max(axis=0)-corners.min(axis=0)) * .2)
        group = next((g for g in groups if np.linalg.norm(g[0][2].mean(axis=0)-center) < radius), None)
        if group is None:
            group = []
            groups.append(group)
        group.append((scale, identity, corners))
    result = []
    for group in groups:
        ids = {item[1] for item in group}
        if len(ids) != 1 or len({item[0] for item in group}) < 2:
            continue
        result.append({'id': int(next(iter(ids))),
                       'corners': np.median([item[2] for item in group], axis=0).tolist(),
                       'agreeing_scales': sorted({item[0] for item in group})})
    for tag in result:
        # Unknown decoded IDs remain visible for diagnostics, never assigned a scale.
        tag['mission_marker'] = marker_spec(tag['id']) if 0 <= tag['id'] <= 14 else None
        tag['pose_verified'] = False
    return result


def detect_tags(image, detector):
    import cv2
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    observations = []
    for scale in (1, 2, 3, 4):
        scaled = gray if scale == 1 else cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detector.detectMarkers(scaled)
        if ids is not None:
            observations.extend((scale, int(identity), corner[0]/scale) for corner, identity in zip(corners, ids.flatten()))
    return agreed_tags(observations)


def tag_evidence(observations):
    """Preserve conflicting/unconfirmed groups that the legacy list omits."""
    groups = []
    for scale, identity, corners in observations:
        corners = np.asarray(corners, float)
        if corners.shape != (4, 2) or not np.isfinite(corners).all():
            raise ValueError("invalid_marker_corners")
        center = corners.mean(axis=0)
        radius = max(3., np.linalg.norm(corners.max(axis=0)-corners.min(axis=0))*.2)
        group = next((g for g in groups if np.linalg.norm(g[0][2].mean(axis=0)-center)<radius),None)
        if group is None:
            group = []; groups.append(group)
        group.append((scale,int(identity),corners))
    diagnostics = []
    for group in groups:
        ids = sorted({x[1] for x in group})
        scales = sorted({x[0] for x in group})
        reason = "agreed"
        if len(ids)!=1:reason="conflicting_scale_ids"
        elif len(scales)<2:reason="insufficient_scale_agreement"
        elif not 0<=ids[0]<=14:reason="unregistered_id"
        diagnostics.append(dict(ids=ids,scales=scales,reason=reason,
                                center=np.median([x[2].mean(axis=0) for x in group],axis=0).tolist()))
    return dict(tags=agreed_tags(observations),groups=diagnostics,
                ambiguous=any(g["reason"]!="agreed" for g in diagnostics))


def detect_tag_evidence(image, detector):
    import cv2
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
    observations = []
    for scale in (1,2,3,4):
        scaled = gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detector.detectMarkers(scaled)
        if ids is not None:
            observations.extend((scale,int(identity),corner[0]/scale)
                                for corner,identity in zip(corners,ids.flatten()))
    return tag_evidence(observations)
