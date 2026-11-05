"""Stationary diagnostic acquisition; fresh robot-only masks and owned outputs."""
import numpy as np


def acquire(cameras, verify_sources=False, profile=False, stream_probe=False):
    import time
    import torch
    import warp as wp
    wall_start=time.perf_counter_ns();raw=[];mask_events=[];mask_wall=[];devices=[]
    for key,rgb,seg,depth in cameras:
        r=rgb.get_data(device='cuda');s=seg.get_data(device='cuda')
        d=None if depth is None else depth.get_data(device='cuda')
        raw.append((key,r,s,d))
    stream_info=None
    if stream_probe:
        producer_nodes=[]
        for key,rgb,seg,depth in cameras:
            for kind,ann in [('rgb',rgb),('seg',seg),('depth',depth)]:
                if ann is None:continue
                node=ann.get_node();attrs={}
                for name in ['outputs:cudaStream','outputs:cudaDeviceIndex','outputs:__device','outputs:__do_array_copy']:
                    attrs[name]=str(node.get_attribute(name).get()) if node.get_attribute_exists(name) else None
                producer_nodes.append({'camera':key,'kind':kind,'attrs':attrs})
        current=wp.get_stream('cuda:0')
        stream_info={'warp_current_stream':int(current.cuda_stream or 0),'torch_current_stream':int(torch.cuda.current_stream().cuda_stream),'torch_default_stream':int(torch.cuda.default_stream().cuda_stream),'producer_nodes':producer_nodes}
        t=time.perf_counter_ns();wp.synchronize_stream(current)
        stream_info['current_stream_wait_wall_ms']=(time.perf_counter_ns()-t)/1e6
    # Replicator/Warp and torch need an explicit cross-stream completion boundary.
    annotations_done=time.perf_counter_ns()
    global_wait_start=time.perf_counter_ns()
    wp.synchronize()
    if stream_info is not None:stream_info['remaining_global_wait_wall_ms']=(time.perf_counter_ns()-global_wait_start)/1e6
    sync_done=time.perf_counter_ns()
    if profile:
        gpu_start=torch.cuda.Event(enable_timing=True);gpu_end=torch.cuda.Event(enable_timing=True);gpu_start.record()
    pieces=[];layout=[];offset=0;source_checks=[]
    def append(key,tensor):
        nonlocal offset
        tensor=tensor.contiguous();dtype=np.dtype(str(tensor.dtype).split('.')[-1])
        nbytes=tensor.numel()*tensor.element_size()
        layout.append(dict(key=key,dtype=dtype.str,shape=list(tensor.shape),offset=offset,nbytes=nbytes))
        pieces.append(tensor.view(torch.uint8).reshape(-1));offset+=nbytes
    for key,r,s,d in raw:
        rt=wp.to_torch(r);st=wp.to_torch(s['data'])
        if rt.device.type!='cuda' or st.device.type!='cuda':raise ValueError('Non-CUDA source')
        if tuple(rt.shape)!=(480,640,4) or rt.dtype!=torch.uint8 or tuple(st.shape)!=(480,640):raise ValueError('Unexpected source shape/dtype')
        if verify_sources:source_checks.extend([(rt,rt.clone()),(st,st.clone())])
        ids=[int(k) for k,v in s['info']['idToLabels'].items() if '/R12/' in str(v)]
        if profile:
            start=torch.cuda.Event(enable_timing=True);end=torch.cuda.Event(enable_timing=True);start.record();mask_start=time.perf_counter_ns()
        mask=torch.isin(st.to(torch.int64),torch.tensor(ids,device=st.device,dtype=torch.int64))
        if profile:
            end.record();mask_events.append((key,start,end));mask_wall.append((time.perf_counter_ns()-mask_start)/1e6);devices.append({'camera':key,'rgb_device':str(rt.device),'seg_device':str(st.device),'seg_dtype':str(st.dtype),'ids':len(ids)})
        rgb=rt[:,:,:3].clone();rgb[mask]=0;append(key,rgb)
        if d is not None:
            dt=wp.to_torch(d)
            if dt.device.type!='cuda' or tuple(dt.shape)!=(480,640) or dt.dtype!=torch.float32:raise ValueError('Unexpected depth')
            if verify_sources:source_checks.append((dt,dt.clone()))
            depth=dt.clone();depth[mask]=0;append(key[:-len('_left_optical')]+'_depth',depth)
    packed=torch.cat(pieces)
    if profile:gpu_end.record()
    submitted=time.perf_counter_ns()
    host=packed.cpu().numpy()
    copied=time.perf_counter_ns()
    if verify_sources:
        # Byte comparison also handles NaN depth bit patterns exactly.
        if not all(torch.equal(a.contiguous().view(torch.uint8),b.contiguous().view(torch.uint8)) for a,b in source_checks):raise ValueError('Mutated annotation source')
    arrays={e['key']:np.frombuffer(host,dtype=np.dtype(e['dtype']),count=e['nbytes']//np.dtype(e['dtype']).itemsize,offset=e['offset']).reshape(e['shape']).copy() for e in layout}
    unpacked=time.perf_counter_ns()
    timing=None
    if profile:
        gpu_end.synchronize()
        timing={'annotation_wall_ms':(annotations_done-wall_start)/1e6,'warp_sync_wall_ms':(sync_done-annotations_done)/1e6,'mask_clone_pack_submit_wall_ms':(submitted-sync_done)/1e6,'d2h_wait_wall_ms':(copied-submitted)/1e6,'unpack_wall_ms':(unpacked-copied)/1e6,'torch_stream_elapsed_ms':gpu_start.elapsed_time(gpu_end),'mask_stream_elapsed_ms':[{'camera':k,'ms':a.elapsed_time(b)} for k,a,b in mask_events],'mask_submit_wall_ms':mask_wall,'devices':devices,'scope':'Wall intervals vs CUDA stream elapsed; event intervals may include host submission gaps,not pure kernel durations.'}
    return arrays,dict(stream_probe=stream_info,profile=timing,layout=layout,packed_bytes=offset,source_arrays_checked=len(source_checks),source_unchanged=True if verify_sources else None)
