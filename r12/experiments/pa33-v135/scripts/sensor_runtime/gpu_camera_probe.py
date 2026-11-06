"""Read-only CUDA annotation and mask equivalence probe; never controls motion."""
import hashlib
import numpy as np


def probe(cameras, device_log=None):
    import json
    import torch
    import warp as wp
    records=[];packed=[];expected=[];retained=[];devices=[]
    for name,rgb,seg,depth in cameras:
        gpu_rgb=rgb.get_data(device='cuda');gpu_seg=seg.get_data(device='cuda')
        cpu_rgb=rgb.get_data(device='cpu').copy();cpu_seg=seg.get_data(device='cpu')
        rg=wp.to_torch(gpu_rgb);sg=wp.to_torch(gpu_seg['data'])
        entry={'camera':name,'rgb_device':str(rg.device),'seg_device':str(sg.device),'rgb_dtype':str(rg.dtype),'seg_dtype':str(sg.dtype),'rgb_shape':list(rg.shape),'seg_shape':list(sg.shape),'node_attributes':{}}
        for label,ann in [('rgb',rgb),('seg',seg),('depth',depth)]:
            if ann is None:continue
            node=ann.get_node();entry['node_attributes'][label]={}
            for attr in ['outputs:cudaDeviceIndex','outputs:__device','outputs:__do_array_copy']:
                if node.get_attribute_exists(attr):entry['node_attributes'][label][attr]=str(node.get_attribute(attr).get())
        devices.append(entry)
        if device_log is not None:device_log.write_text(json.dumps(devices,indent=2)+'\n')
        if rg.device.type!='cuda' or sg.device.type!='cuda':raise ValueError('CUDA output required: '+str(entry))
        source_rgb=rg.clone();source_seg=sg.clone()
        assert rg.cpu().numpy().tobytes()==cpu_rgb.tobytes()
        assert sg.cpu().numpy().tobytes()==cpu_seg['data'].tobytes()
        assert gpu_seg['info']['idToLabels']==cpu_seg['info']['idToLabels']
        ids=[int(k) for k,v in cpu_seg['info']['idToLabels'].items() if '/R12/' in str(v)]
        mask=torch.isin(sg.to(torch.int64),torch.tensor(ids,device=sg.device,dtype=torch.int64))
        cmask=np.isin(cpu_seg['data'],ids)
        assert np.array_equal(mask.cpu().numpy(),cmask)
        masked=rg[:,:,:3].clone();masked[mask]=0
        cpu_masked=cpu_rgb[:,:,:3].copy();cpu_masked[cmask]=0
        assert masked.cpu().numpy().tobytes()==cpu_masked.tobytes()
        packed.append(masked.contiguous().view(torch.uint8).reshape(-1));expected.append(cpu_masked.tobytes())
        retained.append((masked,hashlib.sha256(cpu_masked.tobytes()).hexdigest()))
        if depth is not None:
            gd=wp.to_torch(depth.get_data(device='cuda'));cd=depth.get_data(device='cpu').copy();original_depth=gd.clone()
            entry['depth_device']=str(gd.device);entry['depth_dtype']=str(gd.dtype);entry['depth_shape']=list(gd.shape)
            if device_log is not None:device_log.write_text(json.dumps(devices,indent=2)+'\n')
            assert gd.device.type=='cuda' and gd.cpu().numpy().tobytes()==cd.tobytes()
            md=gd.clone();md[mask]=0;cd[cmask]=0
            assert md.cpu().numpy().tobytes()==cd.tobytes()
            assert gd.cpu().numpy().tobytes()==original_depth.cpu().numpy().tobytes()
            packed.append(md.contiguous().view(torch.uint8).reshape(-1));expected.append(cd.tobytes())
            retained.append((md,hashlib.sha256(cd.tobytes()).hexdigest()))
        assert torch.equal(rg,source_rgb) and torch.equal(sg,source_seg)
        records.append({'camera':name,'rgb_device':str(rg.device),'seg_device':str(sg.device),'rgb_dtype':str(rg.dtype),'seg_dtype':str(sg.dtype),'rgb_shape':list(rg.shape),'mask_pixels':int(cmask.sum()),'cpu_gpu_byte_equal':True,'source_unchanged':True})
    wire=torch.cat(packed).cpu().numpy().tobytes()
    assert wire==b''.join(expected)
    return {'cameras':records,'packed_bytes':len(wire),'packed_byte_equal':True,'scope':'One-frame CUDA read/mask/pack probe, not performance evidence'},retained


def verify_retained(retained):
    return all(hashlib.sha256(t.cpu().numpy().tobytes()).hexdigest()==digest for t,digest in retained)
