"""Atomic publication and stopped-state loading of completed simulation policies."""
from pathlib import Path
import hashlib,json,os,time
import numpy as np
import torch
from .swerve_highspeed_v2_rl import Policy
from .swerve import wheel_odometry

def publish_completed(checkpoint, directory, expected_iteration):
    checkpoint=Path(checkpoint);directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    data=torch.load(checkpoint,map_location='cpu',weights_only=True)
    if data['iteration']!=expected_iteration:raise ValueError('Incomplete training checkpoint')
    model=Policy();model.load_state_dict(data['model'],strict=True)
    if not all(torch.isfinite(v).all() for v in model.state_dict().values()):raise ValueError('Nonfinite weights')
    raw=checkpoint.read_bytes();digest=hashlib.sha256(raw).hexdigest();dest=directory/(digest+'.pt')
    tmp=dest.with_suffix('.tmp');tmp.write_bytes(raw);os.replace(tmp,dest)
    manifest={'schema':'r12-wheel-residual-v2','sha256':digest,'file':dest.name,'iteration':expected_iteration,'published_at':time.time(),'simulation_only':True,'performance_accepted':False,'actuator_contract':data.get('actuator_contract','legacy_alignment_stop')}
    tmp=directory/'latest.tmp';tmp.write_text(json.dumps(manifest,indent=2));os.replace(tmp,directory/'latest.json')
    return manifest

class LatestPolicy:
    def __init__(self,directory):
        self.directory=Path(directory);self.model=None;self.version=None;self.previous=np.zeros(4);self.error=None;self.pending=None;self.count=0
    def refresh(self,stopped):
        if not stopped and self.model is not None:return
        try:
            manifest=json.loads((self.directory/'latest.json').read_text())
            if manifest['schema']!='r12-wheel-residual-v2' or not manifest['simulation_only']:raise ValueError('Incompatible policy manifest')
            if manifest['sha256']==self.version:return
            name=manifest['file']
            if Path(name).name!=name:raise ValueError('Invalid checkpoint filename')
            path=self.directory/name
            if hashlib.sha256(path.read_bytes()).hexdigest()!=manifest['sha256']:raise ValueError('Checkpoint checksum mismatch')
            data=torch.load(path,map_location='cpu',weights_only=True);model=Policy();model.load_state_dict(data['model'],strict=True)
            if not all(torch.isfinite(v).all() for v in model.state_dict().values()):raise ValueError('Nonfinite policy')
            model.eval();self.model=model;self.version=manifest['sha256'];self.previous[:]=0;self.error=None
        except Exception as e:self.error=str(e)
    def action(self,command,steer,wheels,gyro,accel):
        self.count+=1
        stopped=np.linalg.norm(command)<1e-5 and np.max(np.abs(wheels))<.2
        if self.model is None or (stopped and self.count%10==0):self.refresh(stopped)
        if self.model is None:return np.zeros(4)
        command=np.asarray(command);gyro=np.asarray(gyro)
        try:estimate,_=wheel_odometry(steer,wheels)
        except ValueError:self.previous[:]=0;return self.previous.copy()
        estimate[2]=gyro[2]
        obs=np.r_[(command-estimate)/[.9,.9,1.5],command/[.9,.9,1.5],np.asarray(steer)/(np.pi/2),np.asarray(wheels)/29.321531434,gyro/2,np.asarray(accel)/10,self.previous]
        if not np.isfinite(obs).all() or np.linalg.norm(command)<1e-6:self.previous[:]=0;return self.previous.copy()
        with torch.no_grad():self.previous=self.model(torch.tensor(obs,dtype=torch.float32)[None].clamp(-10,10))[0].mean.clamp(-1,1)[0].numpy()
        return self.previous.copy()

def allocate(command,current,previous,action,dt=.05,steer_rate=7.330382858,low_speed_residual_fraction=None):
    if not np.isfinite(steer_rate) or not 0<steer_rate<=7.330382858:raise ValueError("Invalid steering reference rate")
    from .swerve import OFFSETS,BASE_DIRECTIONS,RADIUS
    command=np.asarray(command);current=np.asarray(current)
    if np.linalg.norm(command)<1e-6:return current.copy(),np.zeros(4)
    vectors=command[:2]+command[2]*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])
    peak=np.max(np.linalg.norm(vectors,axis=1));vectors*=min(1,.899795667/max(peak,1e-9))
    raw=(np.arctan2(vectors[:,1],vectors[:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
    angle=np.where(raw>np.pi/2,raw-np.pi,np.where(raw< -np.pi/2,raw+np.pi,raw))
    previous=current if previous is None else np.asarray(previous)
    hold=(np.abs(np.abs(raw)-np.pi/2)<.06)&(np.abs(previous)>.7);angle[hold]=np.sign(previous[hold])*np.pi/2
    heading=BASE_DIRECTIONS+angle;speed=np.sum(vectors*np.column_stack([np.cos(heading),np.sin(heading)]),axis=1)/RADIUS
    error=angle-current;target=previous+np.clip(angle-previous,-steer_rate*dt,steer_rate*dt)
    # Gate the complete command, including PPO residual, per wheel.
    # One misaligned module must not stop the other three.
    residual=6*np.asarray(action)
    if low_speed_residual_fraction is not None:
        from .low_speed_residual import limit_low_speed_residual
        residual=limit_low_speed_residual(speed,residual,peak,low_speed_residual_fraction)
    wheels=(speed+residual)*np.maximum(0,np.cos(error))
    return np.clip(target,-np.pi/2,np.pi/2),np.clip(wheels,-29.321531434,29.321531434)

def dc_effort(target,position,velocity,steering=False,wheel_kd=.02,wheel_feedforward=0.):
    velocity=np.asarray(velocity)
    no_load,stall,continuous=(7.330382858,2.7,.54) if steering else (29.321531434,.54,.108)
    demand=5*(np.asarray(target)-position)-.2*velocity if steering else wheel_kd*(np.asarray(target)-velocity)+wheel_feedforward
    upper=np.minimum(continuous,stall*(1-velocity/no_load));lower=np.maximum(-continuous,stall*(-1-velocity/no_load))
    return np.clip(demand,lower,upper)
