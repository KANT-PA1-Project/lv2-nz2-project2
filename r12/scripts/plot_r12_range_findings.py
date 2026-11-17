#!/usr/bin/env python3
"""Plot measured geometry samples and URDF-based static gravity estimate."""
import json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
x=json.loads((root/'reports/local/r12_geometry_sweep.json').read_text())
fig,axes=plt.subplots(1,2,figsize=(13,4.8),layout='constrained')
ax=axes[0]
for i,(name,rows) in enumerate(x['joints'].items(),1):
 for bad,color,label in [(False,'#15803d','No nonadjacent intersection'),(True,'#dc2626','Intersection detected')]:
  degrees=[r['deg'] for r in rows if bool(r['nonadjacent_intersections'])==bad]
  ax.scatter(degrees,[i]*len(degrees),c=color,s=16,label=label if i==1 else None)
ax.axvspan(-20,20,color='#64748b',alpha=.12,label='Original J2-J5 range (approx.)')
ax.set(xlim=(-185,185),yticks=range(1,6),yticklabels=[f'Joint {i}' for i in range(1,6)],xlabel='Joint angle (degrees)',title='Original STL: 5-degree single-joint sweep')
ax.invert_yaxis();ax.grid(axis='x',alpha=.2);ax.legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.15))
# For joint2 with all other joint angles zero, gravity torque is sinusoidal.
r=json.loads((root/'reports/local/r12_joint2_gravity.json').read_text());a=r[0]['gravity_torque_Nm'];q=math.radians(r[1]['joint2_deg']);b=(a*math.cos(q)-r[1]['gravity_torque_Nm'])/math.sin(q)
deg=np.linspace(-90,90,361);tau=a*np.cos(np.radians(deg))-b*np.sin(np.radians(deg))
ax=axes[1];ax.plot(deg,tau,label='Static gravity estimate from supplied URDF',color='#2563eb');ax.axhline(2,color='#dc2626',ls='--',label='Source drive maximum: 2 N m');ax.fill_between(deg,2,tau,where=tau>2,color='#dc2626',alpha=.2)
ax.set(xlabel='Joint 2 angle (degrees)',ylabel='Gravity torque (N m)',title='Angle expansion also requires enough drive torque');ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper center',bbox_to_anchor=(.5,-.15))
fig.suptitle('Other joints at zero; adjacent geometry contacts excluded. Not hardware travel certification.',fontsize=10)
fig.savefig(root.parent.parent/'outputs/r12/R12_range_findings.png',dpi=160)
