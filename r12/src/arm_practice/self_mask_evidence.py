"""Owned raw/masked/explicit-self-mask evidence; never a clearance certificate.

The caller must supply the mask it actually used and that call's segmentation.
This records one readback transaction, not a cross-annotator native epoch proof.
A future native adapter must bind every annotator's ReferenceTime plus the raw
SHA to its trusted issuer; these diagnostic records must not be promoted.
"""
from dataclasses import dataclass
import hashlib
import json
import re

import numpy as np

_MAX_PIXELS = 3_000_000
_CAMERAS = ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera')
_CLOCKS = ('synthetic_packet_20hz', 'stationary_renderer_diagnostic')


def array_sha256(array):
    """Hash exact C-order bits, retaining NaN payloads and invalid returns."""
    return hashlib.sha256(memoryview(np.ascontiguousarray(array)).cast('B')).hexdigest()


def _uint(value, name):
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError('invalid ' + name)
    return value


def _digest(value, name):
    if not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('invalid ' + name)
    return value


@dataclass(frozen=True)
class SelfMaskEvidence:
    raw: np.ndarray
    masked: np.ndarray
    self_mask: np.ndarray
    metadata_json: str

    def archive_fields(self):
        """New diagnostic keys only: existing perception depths are unchanged."""
        camera = json.loads(self.metadata_json)['camera']
        return {
            camera + '_depth_unmasked': self.raw,
            camera + '_depth_masked_evidence': self.masked,
            camera + '_self_mask': self.self_mask,
            camera + '_self_mask_evidence_json': np.array(self.metadata_json),
        }


def preserve_self_mask_pair(raw, masked, explicit_mask, instance_ids,
                            self_instance_ids, *, run_id, frame_index, camera,
                            clock_domain, stamp_ns, capture_monotonic_ns,
                            readback_start_monotonic_ns, readback_end_monotonic_ns,
                            calibration_sha256, source_sha256):
    """Copy and validate actual callsite arrays without inferring masks from zero.

    Exact mask provenance is checked against the actual segmentation ID array.
    Validation does not establish that segmentation/depth rendered at one native
    epoch: metadata explicitly withholds that authority even for matching bytes.
    """
    if camera not in _CAMERAS or clock_domain not in _CLOCKS:
        raise ValueError('unsupported camera/clock; no native epoch authority')
    if not isinstance(run_id, str) or not 1 <= len(run_id) <= 160 or not re.fullmatch(r'[\w.-]+', run_id):
        raise ValueError('invalid run_id')
    for name, value in (('frame_index', frame_index), ('stamp_ns', stamp_ns),
                        ('capture_monotonic_ns', capture_monotonic_ns),
                        ('readback_start_monotonic_ns', readback_start_monotonic_ns),
                        ('readback_end_monotonic_ns', readback_end_monotonic_ns)):
        _uint(value, name)
    if not 0 < capture_monotonic_ns <= readback_start_monotonic_ns <= readback_end_monotonic_ns:
        raise ValueError('nonmonotonic capture/readback interval')
    _digest(calibration_sha256, 'calibration_sha256')
    _digest(source_sha256, 'source_sha256')
    arrays = (raw, masked, explicit_mask, instance_ids)
    if any(not isinstance(a, np.ndarray) for a in arrays):
        raise ValueError('all four actual arrays are required')
    if raw.ndim != 2 or not 1 <= raw.size <= _MAX_PIXELS or any(a.shape != raw.shape for a in arrays):
        raise ValueError('invalid/mismatched bounded image shape')
    if raw.dtype != np.dtype('float32') or masked.dtype != np.dtype('float32') or explicit_mask.dtype != np.dtype('bool'):
        raise ValueError('raw/masked float32 and explicit bool mask required')
    if instance_ids.dtype.kind not in 'ui' or instance_ids.dtype.itemsize > 8:
        raise ValueError('integer instance IDs required')
    if not isinstance(self_instance_ids, (list, tuple)) or len(self_instance_ids) > 4096:
        raise ValueError('bounded explicit self instance IDs required')
    if any(type(i) is not int or i < 0 or i >= 2**63 for i in self_instance_ids):
        raise ValueError('invalid self instance ID')
    selected = sorted(set(self_instance_ids))
    # Copy all inputs before verification. No source ownership or alias is reused.
    r = np.array(raw, copy=True, order='C')
    m = np.array(masked, copy=True, order='C')
    b = np.array(explicit_mask, copy=True, order='C')
    seg = np.array(instance_ids, copy=True, order='C')
    if not np.array_equal(b, np.isin(seg, selected)):
        raise ValueError('explicit mask does not match the supplied segmentation IDs')
    expected = r.copy()
    expected[b] = 0
    if not np.array_equal(expected.view(np.uint32), m.view(np.uint32)):
        raise ValueError('masked bytes do not equal actual raw with explicit mask')
    meta = dict(schema='r12_self_mask_pair_v1', run_id=run_id,
                frame_index=frame_index, camera=camera,
                camera_frame=camera + '_left_optical', clock_domain=clock_domain,
                stamp_ns=stamp_ns, capture_monotonic_ns=capture_monotonic_ns,
                readback_start_monotonic_ns=readback_start_monotonic_ns,
                readback_end_monotonic_ns=readback_end_monotonic_ns,
                shape=list(r.shape), raw_dtype=r.dtype.str, mask_dtype=b.dtype.str,
                raw_sha256=array_sha256(r), masked_sha256=array_sha256(m),
                self_mask_sha256=array_sha256(b), segmentation_sha256=array_sha256(seg),
                segmentation_dtype=seg.dtype.str, self_instance_ids=selected,
                calibration_sha256=calibration_sha256, source_sha256=source_sha256,
                masked_pixel_count=int(b.sum()),
                pairing_scope='same_callsite_arrays_no_cross_annotator_epoch_certificate',
                native_epoch_verified=False, encoder_association_verified=False,
                issuer_certificate=None, motion_authorized=False, live_confirmed=False,
                interpretation='Self mask is provenance only; never free space at or behind raw return.')
    for a in (r, m, b):
        a.flags.writeable = False
    return SelfMaskEvidence(r, m, b, json.dumps(meta, sort_keys=True, separators=(',', ':'), allow_nan=False))
