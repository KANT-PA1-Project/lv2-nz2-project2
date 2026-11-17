"""Diagnostic identity only; repeated faces are not distinct object identities."""
import math

def diagnostic_identity(objects,*,fresh):
    if fresh is not True or not isinstance(objects,list):
        return None
    candidates=[]
    for obj in objects:
        if not isinstance(obj,dict) or not isinstance(obj.get('pose',{}),dict):
            return None
        if obj.get('pose',{}).get('valid') is not True:
            continue
        depth=obj.get('depth_surface_m')
        if isinstance(depth,bool) or not isinstance(depth,(int,float)) or not math.isfinite(depth) or not .02<depth<10:
            continue
        ids=obj.get('tag_ids',[])
        if not isinstance(ids,list) or not ids or any(type(i)is not int or not 0<=i<12 for i in ids):
            return None
        distinct=set(ids)
        if len(distinct)!=1:
            return None
        candidates.append(next(iter(distinct)))
    # Multiple object boxes remain ambiguous even if they share one ID.
    return candidates[0] if len(candidates)==1 else None
