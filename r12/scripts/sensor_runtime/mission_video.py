"""Record existing camera RGB views without additional rendering or control input."""
import cv2
import subprocess
import time
import imageio_ffmpeg
import numpy as np

class MissionVideo:
    def __init__(self, path):
        self.writer = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '1280x1080', '-r', '20', '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '25', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)
        self.closed = False
        self.frames = 0
        self.label = str(path.parent.name)
        self.started = time.monotonic()

    def write(self, rgb, extras, frame, response):
        names = ['top1', 'top2', 'top3', 'arm']
        views = [rgb[0]] + [extras[k] for k in ['r07_camera2_left_optical', 'r07_camera3_left_optical', 'arm_camera_left_optical']]
        canvas = np.zeros((1080, 1280, 3), dtype=np.uint8)
        for i, (name, view) in enumerate(zip(names, views)):
            tile = cv2.cvtColor(cv2.resize(view, (640,480)), cv2.COLOR_RGB2BGR)
            proof=response.get('wrist_perception',{})
            if name=='arm' and proof.get('fresh') and proof.get('stamp_ns')==response.get('stamp_ns'):
                for detection in proof.get('detections',[]):
                    a,b,c,d=[int(v) for v in detection['bbox']]
                    cv2.rectangle(tile,(a,b),(c,d),(0,255,0),1)
                    label=f'{detection["class_name"]} {detection["confidence"]:.2f}'
                    cv2.putText(tile,label,(a,max(16,b-5)),cv2.FONT_HERSHEY_SIMPLEX,.42,(0,255,0),1)
                for tag in proof.get('tags',[]):
                    corners=np.array(tag['corners'],dtype=np.int32)
                    cv2.polylines(tile,[corners],True,(255,255,0),2)
                    cv2.putText(tile,'ID '+str(tag['id']),tuple(corners[0]),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,0),1)
            cv2.putText(tile, name, (12,28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
            y, x = (i//2)*480, (i%2)*640
            canvas[y:y+480,x:x+640] = tile
        command = response.get('command', [0, 0, 0])
        reasons = [str(x) for x in [response.get('lidar_guard'), response.get('motion_envelope', {}).get('reason')] if x]
        lines = [
            f'{self.label} | sim {frame*.05:.2f}s | recording wall {time.monotonic()-self.started:.1f}s | approaches {len(response.get("completed", []))}/{response.get("total_goals", 10)}',
            f'COMMAND vx {command[0]:+.3f} vy {command[1]:+.3f} m/s yaw {command[2]:+.3f} rad/s | guard: {", ".join(reasons) or "none"}',
            f'VSLAM: {response.get("odom_received", False)} | IMU: {response.get("imu_fused", False)} | encoders: {response.get("encoder_valid", False)} | arm: {response.get("observation_state", response.get("transport_state", "unknown"))}',
            f'SIMULATION inspection | confirmed wrist IDs: {response.get("recognized_puck_ids", [])} | no pick/place | playback: simulation time'
        ]
        for i, text in enumerate(lines):
            cv2.putText(canvas, text[:155], (12, 983+28*i), cv2.FONT_HERSHEY_SIMPLEX, .52, (255,255,255), 1)
        self.writer.stdin.write(canvas.tobytes())
        self.frames += 1

    def close(self):
        if not self.closed:
            self.closed = True
            self.writer.stdin.close()
            if self.writer.wait() != 0:
                raise RuntimeError('Video encoder failed')
