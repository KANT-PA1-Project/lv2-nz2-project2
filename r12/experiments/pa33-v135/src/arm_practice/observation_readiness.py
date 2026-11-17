"""Offline evidence gates. Passing is neither motion permission nor pickup proof.

Thresholds are diagnostic defaults, not calibrated physical operating limits.
The caller must supply the current packet and same-host monotonic clock.
"""
import math
from .perception_response_identity import check_packet_perception


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def box(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError('bbox')
    if not all(finite(x) for x in value) or not (value[0] < value[2] and value[1] < value[3]):
        raise ValueError('bbox')
    return value


def iou(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    return intersection / ((a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])-intersection)


def check_observation(packet, detection_index, *, now_ns, max_age_ns):
    """Reject incomplete evidence; never trust cached sender gate verdicts.

    Accepted tag records come from the existing multiscale detector. This checks
    their structure, not authenticity or independent image-level correctness.
    Cross-class overlap >= .5 is ambiguous; confidence does not break the tie.
    """
    result = dict(evidence_ready=False, execution_authorized=False)
    provenance = check_packet_perception(packet, now_ns=now_ns, max_age_ns=max_age_ns)
    if not provenance['matched']:
        return dict(result, reason=provenance['reason'])
    return _check_content(packet['perception'], detection_index)


def check_archived_content(response, detection_index):
    """Content-only audit: archived records cannot establish current freshness."""
    content = _check_content(response, detection_index)
    return dict(evidence_ready=False, execution_authorized=False,
                live_freshness_verified=False, content_ready=content['evidence_ready'],
                content_reason=content['reason'], identity_evidence=content.get('identity_evidence'),
                reason='archived_freshness_unverified')


def _check_content(response, detection_index):
    result = dict(evidence_ready=False, execution_authorized=False)
    try:
        detections = response['detections']
        if type(detection_index) is not int or detection_index < 0:
            raise ValueError('index')
        d = detections[detection_index]
        b = box(d['bbox'])
        cls = d['class_id']
        if type(cls) is not int or not 0 <= cls < 4 or not finite(d['confidence']) or not .25 <= d['confidence'] <= 1:
            return dict(result, reason='invalid_detection')
        for other in detections:
            ob = box(other['bbox'])
            oc = other['class_id']
            if type(oc) is not int or not finite(other['confidence']):
                raise ValueError('other detection')
            if 0 <= oc < 4 and oc != cls and other['confidence'] >= .25 and iou(b, ob) >= .5:
                return dict(result, reason='class_overlap_ambiguous')
        identity = check_tag_identity(response['tags'], b)
        result['identity_evidence'] = identity
        if not identity['identity_consistent']:
            return dict(result, reason=identity['reason'])
        if identity['face_count'] > 1:
            return dict(result, reason='multi_face_object_association_unverified')
        pose = d['pose']
        center = pose['center_camera_m']
        depth = d['depth_surface_m']
        if (pose['translation_valid'] is not True or not finite(pose['confidence'])
                or not .25 <= pose['confidence'] <= 1 or len(center) != 3
                or not all(finite(x) for x in center) or center[2] <= 0
                or not finite(depth) or not .02 < depth < 10):
            return dict(result, reason='invalid_pose_or_depth')
        if abs(depth-center[2]) >= .1:
            return dict(result, reason='depth_mismatch')
        return dict(result, evidence_ready=True, reason='diagnostic_evidence_only', tag_id=identity['tag_id'])
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return dict(result, reason='malformed_evidence')


def check_tag_identity(tags, detection_bbox):
    """Describe ID agreement without merging corners or proving object association.

    Even disjoint same-ID quads inside one box can be separate physical objects.
    Keep all faces, and never derive shape or pose from their number or IDs.
    Any positive bbox overlap is conservatively ambiguous in this diagnostic.
    """
    result = dict(identity_consistent=False, execution_authorized=False,
                  object_association_verified=False, faces=[], face_count=0)
    try:
        b = box(detection_bbox)
        for tag in tags:
            corners = tag['corners']
            if not isinstance(corners, (list, tuple)) or len(corners) != 4:
                raise ValueError('corners')
            if any(len(p) != 2 or not all(finite(v) for v in p) for p in corners):
                raise ValueError('corners')
            tb = box([min(p[0] for p in corners), min(p[1] for p in corners),
                      max(p[0] for p in corners), max(p[1] for p in corners)])
            if not all(b[0] <= x <= b[2] and b[1] <= y <= b[3] for x, y in corners):
                continue
            scales = tag['agreeing_scales']
            if any(type(v) is not int or v not in (1, 2, 3, 4) for v in scales) or len(set(scales)) < 2:
                return dict(result, reason='tag_evidence_insufficient')
            tid = tag['id']
            if type(tid) is not int or not 0 <= tid < 12:
                return dict(result, reason='invalid_tag_id')
            result['faces'].append(dict(id=tid, corners=[list(p) for p in corners],
                                        agreeing_scales=sorted(set(scales)), bbox=tb))
            result['face_count'] = len(result['faces'])
        if not result['faces']:
            return dict(result, reason='tag_missing_or_ambiguous')
        ids = {face['id'] for face in result['faces']}
        if len(ids) != 1:
            return dict(result, reason='conflicting_tag_ids')
        for i, face in enumerate(result['faces']):
            if any(iou(face['bbox'], other['bbox']) > 0 for other in result['faces'][i+1:]):
                return dict(result, reason='overlapping_tag_evidence')
        return dict(result, identity_consistent=True, tag_id=next(iter(ids)),
                    reason='same_id_disjoint_faces' if len(result['faces']) > 1 else 'single_tag_face')
    except (KeyError, TypeError, ValueError, IndexError, OverflowError):
        return dict(result, reason='malformed_tag_evidence')
