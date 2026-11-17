#!/usr/bin/env python3
"""Render original STL meshes at representative expanded angles using VTK."""
from pathlib import Path
import math,xml.etree.ElementTree as ET
import numpy as np,trimesh
from arm_practice.kinematics import rotation
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkCommonDataModel import vtkPolyData,vtkCellArray
from vtkmodules.vtkRenderingCore import vtkRenderer,vtkRenderWindow,vtkPolyDataMapper,vtkActor,vtkWindowToImageFilter,vtkTextActor
from vtkmodules.vtkIOImage import vtkPNGWriter
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
root=Path(__file__).resolve().parents[1];r=ET.parse(root/'models/r12/r12_arm.urdf').getroot();joints=r.findall('joint');items=[]
for l in r.findall('link'):
 for visual in l.findall('visual'):
  mesh=trimesh.load_mesh(root/'models/r12'/visual.find('geometry/mesh').get('filename'),process=False)
  pd=vtkPolyData();p=vtkPoints();p.SetData(numpy_to_vtk(np.asarray(mesh.vertices,dtype=np.float32),deep=True));pd.SetPoints(p)
  cells=vtkCellArray();cells.SetCells(len(mesh.faces),numpy_to_vtkIdTypeArray(np.c_[np.full(len(mesh.faces),3),mesh.faces].astype(np.int64).ravel(),deep=True));pd.SetPolys(cells)
  mapper=vtkPolyDataMapper();mapper.SetInputData(pd)
  color=[float(x) for x in visual.find('material/color').get('rgba').split()][:3]
  items.append((l.get('name'),mapper,color))
def transforms(q):
 t={'base_link':np.eye(4)}
 for _ in range(len(joints)):
  for j in joints:
   a,b=j.find('parent').get('link'),j.find('child').get('link')
   if a not in t or b in t:continue
   m=np.eye(4);o=j.find('origin')
   if o is not None:
    m[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');v=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
    m[:3,:3]=rotation([0,0,1],v[2])@rotation([0,1,0],v[1])@rotation([1,0,0],v[0])
   if j.get('name','').startswith('arm_joint'):
    m[:3,:3]=m[:3,:3]@rotation(np.fromstring(j.find('axis').get('xyz'),sep=' '),q[int(j.get('name')[-1])-1])
   t[b]=t[a]@m
 return t
poses=[('Home',[0]*5),('Joint 2: -60 deg',[0,-60,0,0,0]),('Joint 2: +30 deg (mast interference)',[0,30,0,0,0]),('Joint 3: +60 deg',[0,0,60,0,0]),('Joint 4: +100 deg',[0,0,0,100,0]),('Joint 5: +180 deg',[0,0,0,0,180])]
win=vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1800,1200);win.SetMultiSamples(4)
for i,(label,degrees) in enumerate(poses):
 ren=vtkRenderer();ren.SetBackground(.94,.95,.97);col=i%3;row=i//3;ren.SetViewport(col/3,(1-row)/2,(col+1)/3,(2-row)/2);win.AddRenderer(ren)
 ts=transforms([math.radians(x) for x in degrees])
 for name,mapper,color in items:
  a=vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color);a.GetProperty().SetAmbient(.3);a.GetProperty().SetDiffuse(.7)
  m=vtkMatrix4x4()
  for y in range(4):
   for x in range(4):m.SetElement(y,x,float(ts[name][y,x]))
  a.SetUserMatrix(m);ren.AddActor(a)
 text=vtkTextActor();text.SetInput(label);text.SetPosition(15,565);text.GetTextProperty().SetFontSize(16);text.GetTextProperty().SetColor(.1,.1,.15);ren.AddViewProp(text)
 cam=ren.GetActiveCamera();cam.SetPosition(1.5,-1.8,1.15);cam.SetFocalPoint(0,0,.45);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(.56);ren.ResetCameraClippingRange()
win.Render();image=vtkWindowToImageFilter();image.SetInput(win);image.ReadFrontBufferOff();image.Update();writer=vtkPNGWriter();writer.SetFileName(str(root.parent.parent/'outputs/r12/R12_geometry_angles.png'));writer.SetInputConnection(image.GetOutputPort());writer.Write();win.Finalize()
print('Rendered six full-mesh poses')
