"""Pure preparation routing. No simulator, camera or mission reset imports."""
import math


def wall_budget(cfg):
    value = cfg.get('max_wall_seconds', 900.)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 60 <= value <= 1800:
        raise ValueError('max_wall_seconds: finite seconds in [60,1800] required')
    return float(value)


def validate_fallback(cfg, lower, upper):
    spec = cfg.get('signal_fallback')
    if spec is None:
        return
    if not isinstance(spec, dict):
        raise ValueError('signal_fallback must be an object')
    if cfg.get('trial_mode', 'barcode_then_tower') != 'barcode_then_tower' or cfg.get('signal_camera') != 'mast':
        raise ValueError('signal_fallback requires barcode_then_tower and initial mast camera')
    if (cfg['tower_base_xyz_yaw'] != cfg['barcode_base_xyz_yaw']
            or cfg['tower_arm_waypoints_rad'] != [cfg['barcode_arm_q_rad']]):
        raise ValueError('Initial mast attempt must keep the barcode base/arm pose')
    camera = spec.get('camera', 'wrist')
    if camera not in ('mast', 'wrist'):
        raise ValueError('signal_fallback.camera: mast or wrist required')
    if camera == 'mast' and (spec.get('transport_arm_q_rad') is not None
            or spec.get('arm_waypoints_rad') != [cfg['barcode_arm_q_rad']]):
        raise ValueError('Mast relocation must keep the barcode arm pose without transport motion')
    for key in ('allow_partial_sides', 'base_before_arm'):
        if key in spec and not isinstance(spec[key], bool):
            raise ValueError(f'signal_fallback.{key}: boolean required')
    for key in ('initial_prepare_timeout_s', 'fallback_prepare_timeout_s'):
        v = spec.get(key)
        if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v < 1.5:
            raise ValueError(f'signal_fallback.{key}: finite seconds >=1.5 required')
    def vector(value, n):
        return (isinstance(value, list) and len(value) == n and
                all(not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x) for x in value))
    if not vector(spec.get('base_xyz_yaw'), 4):
        raise ValueError('signal_fallback.base_xyz_yaw: four finite numbers required')
    transport = spec.get('transport_arm_q_rad')
    if transport is not None and (not vector(transport, 5) or any(not lo <= x <= hi for x, lo, hi in zip(transport, lower, upper))):
        raise ValueError('signal_fallback transport pose outside configured limits')
    poses = spec.get('arm_waypoints_rad')
    if not isinstance(poses, list) or not poses:
        raise ValueError('signal_fallback.arm_waypoints_rad: nonempty list required')
    for q in poses:
        if not vector(q, 5) or any(not lo <= x <= hi for x, lo, hi in zip(q, lower, upper)):
            raise ValueError('signal_fallback arm waypoint outside configured limits')


class SignalFallback:
    """Switch only before READY; movement time never consumes preparation time."""
    def __init__(self, spec):
        self.spec = spec
        self.mode = 'initial'
        self.camera = 'mast'
        self.started = None
        self.recording = False
        self.events = []

    @property
    def timeout_s(self):
        return self.spec['initial_prepare_timeout_s' if self.mode == 'initial' else 'fallback_prepare_timeout_s']

    def begin(self, t):
        if self.recording or self.started is not None or not math.isfinite(t):
            raise ValueError('Preparation can begin only once per camera, before READY')
        self.started = float(t)
        self.events.append(dict(event='prepare_started', t_s=t, camera=self.camera))

    def timeout_action(self, t, reason):
        if self.recording:
            return None
        if self.started is None:
            raise ValueError('Wait for settling before checking preparation deadline')
        if t < self.started or not math.isfinite(t):
            raise ValueError('Invalid preparation timestamp')
        if t - self.started + 1e-9 < self.timeout_s:
            return None
        if self.mode == 'fallback':
            return 'fail'
        camera = self.spec.get('camera', 'wrist')
        self.events.append(dict(event='fallback_requested', t_s=t, reason=reason,
                                from_camera='mast', to_camera=camera,
                                action='relocate_mast' if camera == 'mast' else 'switch_to_wrist'))
        self.mode, self.camera, self.started = 'fallback', camera, None
        return 'fallback'

    @staticmethod
    def accepts_box(box, shape, tiers=None, margin=3):
        """Allow one side clipped only with three usable observed lens regions.

        Width consistency is evidence of usable visible area, not a measurement
        of the fraction hidden outside the image. Vertical clipping is rejected.
        """
        if box is None or len(box) != 4 or not all(math.isfinite(v) for v in box):
            return False
        x0, y0, x1, y1 = box
        if not (0 <= x0 < x1 <= shape[1] and margin <= y0 < y1 <= shape[0]-margin):
            return False
        left, right = x0 < margin, x1 > shape[1]-margin
        if not left and not right:
            return True
        if left and right or not tiers or set(tiers) != set('RYG'):
            return False
        complete_widths = []
        centers = []
        for c in 'RYG':
            x, y, w, h = tiers[c]
            if (not all(math.isfinite(v) for v in (x, y, w, h)) or w < 8 or h < 8
                    or x < 0 or x+w > shape[1] or y < margin or y+h > shape[0]-margin):
                return False
            centers.append(y+h/2)
            if margin <= x and x+w <= shape[1]-margin:
                complete_widths.append(w)
        if not complete_widths or not centers[0] < centers[1] < centers[2]:
            return False
        reference = max(complete_widths)
        return all(tiers[c][2] >= .6*reference for c in 'RYG')

    def lock(self, t):
        if self.recording or self.started is None:
            raise ValueError('Cannot issue READY without a fresh camera preparation')
        self.recording = True
        self.events.append(dict(event='ready', t_s=t, camera=self.camera))

    def snapshot(self):
        return dict(mode=self.mode, camera=self.camera, fallback_used=self.mode == 'fallback',
                    ready_issued=self.recording, events=list(self.events))
