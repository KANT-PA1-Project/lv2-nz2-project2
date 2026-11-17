from pathlib import Path
import json,math,sys
import numpy as np
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg as ff
out=Path(__file__).resolve().parent
m=mj.MjModel.from_xml_path(str(out/'scene.xml'));d=mj.MjData(m)
movie='--video' in sys.argv
names=[m.actuator(i).name for i in range(m.nu)]
def smooth(t,a,b):
 u=np.clip((t-a)/(b-a),0,1);return u*u*(3-2*u)
def wave(t,a,b):
 return math.sin(math.pi*float(np.clip((t-a)/(b-a),0,1)))**2
def cmd(t):
 steer=.72*smooth(t,2.7,3.7)*(1-smooth(t,5.8,6.8))
 speed=5.5*wave(t,.3,2.6)+5.5*wave(t,3.6,5.8)
 arm=wave(t,6.2,11.4);grip=-.02768*wave(t,8.3,11.3)
 arm_targets=dict(arm_yaw=.62*arm,shoulder=.38*arm,elbow=-.65*arm,wrist_pitch=.47*arm,wrist_roll=1.08*arm,jaw_left=grip)
 return [steer if n.endswith('_steer') else speed if n.endswith('_wheel') else arm_targets[n] for n in names]
fps=20;duration=12;steps=round(duration/m.opt.timestep)
if movie:
 W,H=800,480;r=mj.Renderer(m,H,W);cam=mj.MjvCamera();cam.type=mj.mjtCamera.mjCAMERA_FREE
 w=ff.write_frames(str(out/'mujoco_motion.mp4'),(W,H),fps=fps,codec='libx264',output_params=['-crf','21','-movflags','+faststart'],macro_block_size=1);w.send(None)
 try: font=ImageFont.truetype('DejaVuSans.ttf',15)
 except OSError: font=ImageFont.load_default()
 try: title=ImageFont.truetype('DejaVuSans.ttf',20)
 except OSError: title=font
 labels=['01   Wheel drive','02   Swerve steering and crab drive','03   Arm articulation and gripper']
records=[];base=[];contact=[];jawdiff=[];peakroll=0
for step in range(steps):
 t=d.time;d.ctrl[:]=cmd(t);mj.mj_step(m,d)
 assert np.isfinite(d.qpos).all()
 if step%10==0:
    records.append([float(d.joint(n+'_joint').qpos[0]) for n in names+['jaw_right']]);base.append(d.qpos[:3].copy());jawdiff.append(abs(records[-1][-1]-records[-1][-2]))
    if t>1:contact.append(d.ncon)
    peakroll=max(peakroll,float(np.arccos(np.clip(d.body('base_link').xmat.reshape(3,3)[2,2],-1,1))))
 if movie and step%round(1/(fps*m.opt.timestep))==0:
    phase=0 if t<2.7 else 1 if t<6.25 else 2
    center=d.body('base_link').xpos.copy()
    if phase==1:center+=np.array([.03,0,-.085]);cam.distance=.83;cam.azimuth=52;cam.elevation=-9
    elif phase==2:center+=np.array([-.195,0,.18]);cam.distance=.99;cam.azimuth=116;cam.elevation=-17
    else:center+=np.array([-.08,0,.09]);cam.distance=1.48;cam.azimuth=120;cam.elevation=-18
    cam.lookat[:]=center;r.update_scene(d,cam);im=Image.fromarray(r.render());draw=ImageDraw.Draw(im)
    draw.rectangle((0,0,W,44),fill='#15202a');draw.text((18,9),labels[phase],font=title,fill='white');draw.text((W-115,12),f'{t:04.1f} s',font=font,fill='#9edaca')
    steer=float(np.rad2deg(d.joint('front_left_steer_joint').qpos[0]));wheel=float(d.joint('front_left_wheel_joint').qvel[0])
    text=(f'Steering  {steer:+.0f} deg     Wheel  {wheel:+.1f} rad/s     Base travel  {100*np.linalg.norm(d.qpos[:2]):.0f} cm' if phase<2 else '   '.join(f'{label} {np.rad2deg(d.joint(n+"_joint").qpos[0]):+.0f}' for n,label in zip(names[8:13],['Yaw','Shoulder','Elbow','Pitch','Roll']))+f'    Jaw {1000*d.joint("jaw_left_joint").qpos[0]:.1f} mm')
    draw.rectangle((0,H-33,W,H),fill='#15202a');draw.text((18,H-26),text,font=font,fill='#e5efe9')
    w.send(np.ascontiguousarray(im))
if movie:w.close();r.close()
a=np.array(records);b=np.array(base);span=np.ptp(a,axis=0)
report=dict(mujoco=mj.__version__,duration_s=d.time,timestep_s=m.opt.timestep,steps=steps,controls=m.nu,moving_joints=len(span),joint_ranges={n:[round(x,6),round(y,6)] for n,x,y in zip(names+['jaw_right'],a.min(0),a.max(0))},base_travel_m=float(np.linalg.norm(b[-1,:2]-b[0,:2])),maximum_base_tilt_deg=float(np.rad2deg(peakroll)),max_jaw_mismatch_m=max(jawdiff),minimum_contacts_after_1s=min(contact),solver_warnings=[int(g.number) for g in d.warning])
print(json.dumps(report));assert np.all(span[:13]>.15) and min(span[13:])>.027 and report['base_travel_m']>.12 and max(jawdiff)<.001 and peakroll<.18 and not any(report['solver_warnings'])
(out/'verification.json').write_text(json.dumps(report,indent=2))


