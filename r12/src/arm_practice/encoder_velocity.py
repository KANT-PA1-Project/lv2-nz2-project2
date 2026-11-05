"""Wrapped encoder difference for intervals shorter than half a revolution."""
import numpy as np

def wrapped_velocity(current,previous,dt):
    current=np.asarray(current,float);previous=np.asarray(previous,float)
    if current.shape!=previous.shape or not np.isfinite(np.r_[current.ravel(),previous.ravel(),dt]).all() or dt<=0:raise ValueError('Invalid encoder samples')
    return ((current-previous+np.pi)%(2*np.pi)-np.pi)/dt
