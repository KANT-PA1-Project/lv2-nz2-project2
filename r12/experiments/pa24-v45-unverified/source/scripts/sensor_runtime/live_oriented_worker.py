"""One immutable live-slice request; no ROS/Isaac initialization in this child."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import distance_transform_edt
from oriented_path_adapter import plan_oriented_path, uncertainty_footprint
from live_map_revalidation import slice_snapshot, footprint_evidence


def run(directory):
    request = json.loads((directory/'request.json').read_text())
    with np.load(directory/'slice.npz') as arrays:
        occupied=arrays['occupied'].copy()
        grid=slice_snapshot(request['map_metadata'],arrays['raw_data'])
    if grid['revision']!=request['map_revision'] or not np.array_equal(occupied,grid['occupied']):
        raise ValueError('Archived live slice revision/mask mismatch')
    resolution=float(request['resolution'])
    origin=np.asarray(request['origin'][:2],float)
    pose=np.asarray(request['sensor_pose'],float);pose[:2]-=origin
    path=np.asarray(request['path'],float)-origin
    clearance=distance_transform_edt(~occupied)*resolution
    local=plan_oriented_path(path,pose,request['footprint'],occupied,clearance,resolution,
        position_margin_m=request['position_margin_m'],yaw_error_rad=request['yaw_error_rad'])
    if not local.get('ok'):
        robust=uncertainty_footprint(request['footprint'],request['position_margin_m'],request['yaw_error_rad'])
        local['start_footprint_evidence']=footprint_evidence(request['sensor_pose'],robust,grid)
    plan=dict(local)
    if local.get('ok'):
        plan['path']=(np.asarray(local['path'])+origin).tolist()
        plan['preparation_origin']=(np.asarray(local['preparation_origin'])+origin).tolist()
        plan['goal']=(np.asarray(local['goal'])+origin).tolist()
    tmp=directory/'result.json.tmp'
    tmp.write_text(json.dumps({'request':request,'plan':plan,'plan_local':local},allow_nan=False))
    tmp.replace(directory/'result.json')


if __name__=='__main__':run(Path(sys.argv[1]))
