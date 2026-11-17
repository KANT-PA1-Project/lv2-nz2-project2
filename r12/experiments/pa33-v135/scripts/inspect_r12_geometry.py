#!/usr/bin/env python3
"""Detailed supplied STL triangle intersection sweep; not hardware travel certification."""
import json,math,itertools
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh,fcl
from arm_practice.kinematics import rotation
ROOT=Path(__file__).resolve().parents[1]
r=ET.parse(ROOT/'models/r12/r12_arm.urdf').getroot()
links=['base_link']+[f'arm_link{i}' for i in range(1,6)]
meshes={};objects={}
for name in links:
    link=r.find(f"link[@name='{name}']")
    parts=[trimesh.load_mesh(ROOT/'models/r12'/v.find('geometry/mesh').get('filename'),process=False) for v in link.findall('visual')]
    mesh=trimesh.util.concatenate(parts);meshes[name]=mesh
    model=fcl.BVHModel();model.beginModel(len(mesh.vertices),len(mesh.faces));model.addSubModel(mesh.vertices,mesh.faces);model.endModel();objects[name]=fcl.CollisionObject(model)
    print('loaded',name,len(mesh.faces),flush=True)
joints=r.findall('joint')
def transforms(q):
    t={'base_link':np.eye(4)}
    for i in range(1,6):
        j=r.find(f"joint[@name='arm_joint{i}']");o=j.find('origin');a=np.eye(4)
        a[:3,3]=np.fromstring(o.get('xyz'),sep=' ');axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
        a[:3,:3]=rotation(axis,q[i-1]);t[f'arm_link{i}']=t[j.find('parent').get('link')]@a
    return t
pairs=list(itertools.combinations(links,2))
adjacent={frozenset((j.find('parent').get('link'),j.find('child').get('link'))) for j in joints}
def collisions(q):
    ts=transforms(q)
    for n,o in objects.items():o.setTransform(fcl.Transform(ts[n][:3,:3],ts[n][:3,3]))
    hits=[]
    for a,b in pairs:
        res=fcl.CollisionResult()
        if fcl.collide(objects[a],objects[b],fcl.CollisionRequest(num_max_contacts=1),res):hits.append([a,b])
    return hits
base=collisions([0.]*5);baseline={tuple(p) for p in base}
report={'method':'FCL triangle surface intersection on original visual STL; 5 degree single-joint grid; other joints zero','baseline_intersections':base,'adjacent_pairs':[sorted(x) for x in adjacent if all(n in links for n in x)],'joints':{}}
for j in range(5):
    rows=[]
    for deg in range(-180,181,5):
        q=[0.]*5;q[j]=math.radians(deg);hits=collisions(q)
        nonadj=[p for p in hits if frozenset(p) not in adjacent]
        rows.append({'deg':deg,'intersections':hits,'nonadjacent_intersections':nonadj,'new_pair_intersections':[p for p in hits if tuple(p) not in baseline]})
    report['joints'][f'arm_joint{j+1}']=rows
    (ROOT/'reports/local/r12_geometry_sweep.json').write_text(json.dumps(report,indent=2)+'\n')
    good=[x['deg'] for x in rows if not x['nonadjacent_intersections']]
    print('joint',j+1,'nonadjacent clear samples',good,flush=True)
# Actual STL rendering; face subsampling is for display only, never collision checks.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(16,10))
poses=[('Home',[0]*5),('Joint 2: -90 deg',[0,-math.pi/2,0,0,0]),('Joint 2: +90 deg',[0,math.pi/2,0,0,0]),('Joint 3: +90 deg',[0,0,math.pi/2,0,0]),('Joint 4: +90 deg',[0,0,0,math.pi/2,0]),('Joint 5: +90 deg',[0,0,0,0,math.pi/2])]
for k,(title,q) in enumerate(poses,1):
    ax=fig.add_subplot(2,3,k,projection='3d');ts=transforms(q)
    for i,n in enumerate(links):
        m=meshes[n]
        if len(m.faces)>15000:m=m.simplify_quadric_decimation(face_count=15000)
        faces=m.triangles
        faces=faces@ts[n][:3,:3].T+ts[n][:3,3]
        ax.add_collection3d(Poly3DCollection(faces,facecolor=plt.cm.tab10(i),edgecolor='none',alpha=.6 if i==0 else 1.))
    ax.set(xlim=(-.35,.4),ylim=(-.3,.3),zlim=(0,.9),title=title,xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)');ax.view_init(20,-75);ax.set_box_aspect((.75,.6,.9))
fig.tight_layout();fig.savefig(ROOT.parent.parent/'outputs/r12/R12_geometry_angles.png',dpi=150)
