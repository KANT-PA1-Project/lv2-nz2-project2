"""Estimate puck-pocket centers from printed tags and a timestamp-matched optical pose."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
LAYOUT = json.loads((HERE / 'slots.json').read_text())
SLOTS = {s['id']: s for s in LAYOUT['slots']}
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
PARAM = cv2.aruco.DetectorParameters()
PARAM.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
DETECTOR = cv2.aruco.ArucoDetector(DICT, PARAM)


def on_plane(pixels, K, distortion, base_from_optical, height):
    rays = cv2.undistortPoints(np.asarray(pixels, dtype=float).reshape(-1, 1, 2), K, distortion)
    rays = np.column_stack((rays.reshape(-1, 2), np.ones(len(rays)))) @ base_from_optical[:3, :3].T
    eye = base_from_optical[:3, 3]
    if np.any(np.abs(rays[:, 2]) < 1e-7):
        raise ValueError('Viewing ray is parallel to the floor')
    dist = (height - eye[2]) / rays[:, 2]
    if np.any(dist <= 0):
        raise ValueError('Slot floor is behind the camera')
    return eye + dist[:, None] * rays


def observe(image, K, distortion, pose):
    corners, ids, _ = DETECTOR.detectMarkers(image)
    observations = {}
    for quad, key in zip(corners, [] if ids is None else ids.ravel()):
        key = int(key)
        if key not in SLOTS:
            continue
        floor = SLOTS[key]['floor_center_m'][2]
        xyz = on_plane(quad, K, distortion, pose, floor + LAYOUT['marker_height_above_floor_m'])
        x = (xyz[1] + xyz[2] - xyz[0] - xyz[3])[:2]
        y = (xyz[0] + xyz[1] - xyz[2] - xyz[3])[:2]
        if not .8 < np.linalg.norm(x)/(2*LAYOUT['black_edge_m']) < 1.2:
            continue
        x /= np.linalg.norm(x)
        y -= np.dot(x, y)*x
        y /= np.linalg.norm(y)
        observations[key] = xyz.mean(axis=0), np.column_stack((x,y))
    return observations


def calibrate(key, four_pixels, observation, K, distortion, pose):
    floor = SLOTS[key]['floor_center_m'][2]
    pts = on_plane(four_pixels, K, distortion, pose, floor)[:, :2]
    width = SLOTS[key]['floor_width_mm']*.001
    edges = np.linalg.norm(np.roll(pts,-1,axis=0)-pts,axis=1)
    if np.any(np.abs(edges-width) > .003) or abs(cv2.contourArea(pts.astype('f4'))-width**2) > .15*width**2:
        raise ValueError('Select the four inner FLOOR corners in perimeter order; check camera calibration and tray height.')
    center, axes = observation
    # Both trays are mounted parallel to base_link X in the nominal assembly.
    # Select the pair of horizontal edges nearest +X, independent of click start.
    edge = np.roll(pts,-1,axis=0)-pts
    edge = edge[np.argsort(np.abs(edge[:,0]))[-2:]]
    edge *= np.sign(edge[:,0:1])
    tray_x = edge.mean(axis=0)
    tray_x /= np.linalg.norm(tray_x)
    return dict(center_offset_m=(axes.T @ (pts.mean(axis=0)-center[:2])).tolist(),
                tray_x_in_tag=(axes.T @ tray_x).tolist())


class SlotTracker:
    def __init__(self, offsets):
        self.offsets = {int(k):v for k,v in offsets.items()}
        self.last_seen = {}

    def update(self, image, K, distortion, base_from_optical, stamp):
        seen = observe(image, K, distortion, base_from_optical)
        direct = {}
        for k,(center, axes) in seen.items():
            if k not in self.offsets:
                continue
            calibration = self.offsets[k]
            offset = calibration['center_offset_m'] if isinstance(calibration,dict) else calibration
            xy = center[:2] + axes @ offset
            direct[k] = dict(center_base_m=[*xy,SLOTS[k]['floor_center_m'][2]], stamp=float(stamp),
                             source='direct', source_tag=k, visible=True)
        current = dict(direct)
        for slot,S in SLOTS.items():
            if slot in current:
                continue
            candidates = []
            for k,obs in direct.items():
                calibration = self.offsets[k]
                anchor = np.array(SLOTS[k]['floor_center_m'])
                target = np.array(S['floor_center_m'])
                if anchor[1]*target[1] < 0 or not isinstance(calibration,dict):
                    continue
                x = seen[k][1] @ calibration['tray_x_in_tag']
                tray_rotation = np.array([[x[0],-x[1]],[x[1],x[0]]])
                xy = obs['center_base_m'][:2] + tray_rotation @ (target[:2]-anchor[:2])
                candidates.append((np.linalg.norm(target-anchor),k,xy))
            if candidates:
                _,k,xy = min(candidates,key=lambda c:c[0])
                current[slot] = dict(center_base_m=[*xy,S['floor_center_m'][2]],stamp=float(stamp),
                                     source='rigid_tray',source_tag=k,visible=slot in seen)
        for k,v in list(self.last_seen.items()):
            if k not in current:
                self.last_seen[k] = dict(v, source='cached', visible=False)
        self.last_seen.update(current)
        return {k:dict(v) for k,v in self.last_seen.items()}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('image'); p.add_argument('camera',help='JSON: K, distortion, base_from_optical, stamp')
    p.add_argument('--offsets',required=True,type=Path);p.add_argument('--calibrate',type=int,choices=range(8))
    p.add_argument('--corners',nargs=8,type=float,metavar='PIXEL')
    a=p.parse_args();im=cv2.imread(a.image);cam=json.loads(Path(a.camera).read_text())
    K=np.array(cam['K'],dtype=float).reshape(3,3);D=np.asarray(cam['distortion'],dtype=float);T=np.array(cam['base_from_optical'],dtype=float).reshape(4,4)
    offsets=json.loads(a.offsets.read_text()) if a.offsets.exists() else {}
    if a.calibrate is not None:
        obs=observe(im,K,D,T)
        if a.calibrate not in obs:raise SystemExit('Tag could not be read in the image')
        if a.corners:points=np.asarray(a.corners).reshape(4,2)
        else:
            points=[];title='Click 4 FLOOR corners in perimeter order | Esc: cancel'
            cv2.namedWindow(title)
            cv2.setMouseCallback(title,lambda event,x,y,*rest: points.append((x,y)) if event==cv2.EVENT_LBUTTONDOWN and len(points)<4 else None)
            while len(points)<4:
                preview=im.copy()
                for point in points:cv2.circle(preview,point,3,(0,255,0),-1)
                cv2.imshow(title,preview)
                if cv2.waitKey(20)==27:raise SystemExit('Canceled')
            cv2.destroyAllWindows()
        offsets[str(a.calibrate)]=calibrate(a.calibrate,points,obs[a.calibrate],K,D,T)
        tmp=a.offsets.with_suffix('.tmp');tmp.write_text(json.dumps(offsets,indent=2));tmp.replace(a.offsets)
    print(json.dumps(SlotTracker(offsets).update(im,K,D,T,cam['stamp']),indent=2))
