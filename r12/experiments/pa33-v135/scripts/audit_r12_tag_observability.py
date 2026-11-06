"""Offline rendered-image audit; never supplies fixture identities to control."""
import argparse
import json
import subprocess
import imageio_ffmpeg
from collections import Counter
from pathlib import Path
import cv2
import numpy as np


def decode(detector, image):
    _, ids, rejected = detector.detectMarkers(image)
    return ([] if ids is None else ids.flatten().tolist()), len(rejected)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
    root = Path(__file__).resolve().parents[1]
    textures = {p.name: decode(detector, cv2.imread(str(p)))[0]
                for p in sorted((root/'models/navigation/textures').glob('tag36h11-*.png'))}
    rows = [json.loads(line) for line in (args.run/'perception.jsonl').read_text().splitlines()]
    poses = [d for row in rows for d in row['detections'] if 'pose' in d]
    failures = Counter('valid_translation' if d['pose'].get('translation_valid') else 'invalid_translation' for d in poses)
    errors = [abs(d['depth_surface_m']-d['pose']['center_camera_m'][2]) for d in poses
              if d.get('depth_surface_m') is not None and d['pose'].get('center_camera_m') is not None]
    sizes = [min(d['bbox'][2]-d['bbox'][0],d['bbox'][3]-d['bbox'][1])
             for row in rows for d in row['detections'] if d['class_name']=='apriltag']
    video = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-v', 'error', '-i', str(args.run/'mission.mp4'), '-vf', 'fps=1', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'], stdout=subprocess.PIPE)
    samples = 0
    hits = []
    frame = 0
    while True:
        raw = video.stdout.read(1280*1000*3)
        if not raw:
            break
        if len(raw) != 1280*1000*3:
            raise RuntimeError('Truncated decoded frame')
        mosaic = np.frombuffer(raw,dtype=np.uint8).reshape(1000,1280,3)
        # Recording layout is four existing sensor views, each640x480.
        for camera,(x,y) in enumerate([(0,0),(640,0),(0,480),(640,480)]):
            view = mosaic[y:y+480,x:x+640]
            for variant, image in [('native',view),('upsample2',cv2.resize(view,None,fx=2,fy=2)),
                                    ('mirror2',cv2.resize(cv2.flip(view,1),None,fx=2,fy=2))]:
                ids, rejected = decode(detector,image)
                samples += 1
                if ids:
                    hits.append(dict(frame=frame,camera=camera+1,variant=variant,ids=ids))
        frame += 20
    video.stdout.close()
    if video.wait() or not samples:
        raise RuntimeError('Video decode failed or no frames audited')
    result = dict(run=args.run.name, texture_decodes=textures, video_image_tests=samples,
                  decoded_video_hits=hits, pose_translation_validity=dict(failures),
                  pose_depth_abs_error_m_percentiles=np.percentile(errors,[0,50,95,100]).tolist() if errors else [],
                  detector_tag_short_side_px_percentiles=np.percentile(sizes,[0,50,95,100]).tolist() if sizes else [],
                  limitation='Detector boxes are hypotheses, not verified tag sizes. Video is lossy. No fixture identity supplied to controller; mirroring is diagnostic only.')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
