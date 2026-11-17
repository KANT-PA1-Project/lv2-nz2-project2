"""Owned bit-preserving RGB copy; no buffer reuse or sensor bypass."""
import numpy as np
import cv2

def owned_rgb(raw):
 a=np.asarray(raw)
 if a.dtype==np.uint8 and a.ndim==3 and a.shape[2]==4 and a.flags.c_contiguous:
  result=cv2.cvtColor(a,cv2.COLOR_RGBA2RGB)
 else:result=a[:,:,:3].copy()
 if not result.flags.owndata or np.shares_memory(result,a):raise ValueError('RGB_copy_not_owned')
 return result
