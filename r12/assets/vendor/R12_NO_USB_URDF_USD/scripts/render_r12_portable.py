from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np,trimesh
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkPolyData,vtkCellArray
from vtkmodules.vtkRenderingCore import vtkRenderer,vtkRenderWindow,vtkPolyDataMapper,vtkActor,vtkWindowToImageFilter
from vtkmodules.vtkIOImage import vtkPNGWriter
import vtkmodules.vtkRenderingOpenGL2
from vtkmodules.util.numpy_support import numpy_to_vtk,numpy_to_vtkIdTypeArray
from build_r12_portable import OUT,fk

def actor(mesh,color):
 pd=vtkPolyData();points=vtkPoints();points.SetData(numpy_to_vtk(np.asarray(mesh.vertices,dtype=np.float32),deep=True));pd.SetPoints(points)
 ca=vtkCellArray();ca.SetCells(len(mesh.faces),numpy_to_vtkIdTypeArray(np.c_[np.full(len(mesh.faces),3),mesh.faces].astype(np.int64).ravel(),deep=True));pd.SetPolys(ca)
 mapper=vtkPolyDataMapper();mapper.SetInputData(pd);a=vtkActor();a.SetMapper(mapper);a.GetProperty().SetColor(*color);a.GetProperty().SetAmbient(.25);a.GetProperty().SetDiffuse(.75);return a

def render(items,path,eye=(1.5,-1.8,1.25),target=(0,0,.43),scale=.63,size=(1400,1100)):
 ren=vtkRenderer();ren.SetBackground(.94,.95,.97);win=vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(*size);win.AddRenderer(ren);win.SetMultiSamples(4)
 for m,c in items:ren.AddActor(actor(m,c))
 cam=ren.GetActiveCamera();cam.SetPosition(*eye);cam.SetFocalPoint(*target);cam.SetViewUp(0,0,1);cam.ParallelProjectionOn();cam.SetParallelScale(scale);ren.ResetCameraClippingRange();win.Render()
 img=vtkWindowToImageFilter();img.SetInput(win);img.ReadFrontBufferOff();img.Update();w=vtkPNGWriter();w.SetFileName(str(path));w.SetInputConnection(img.GetOutputPort());w.Write();win.Finalize()
if __name__=='__main__':
 r=ET.parse(OUT/'urdf/r12_no_usb.urdf').getroot();ts=fk(r);items=[]
 for l in r.findall('link'):
  for v in l.findall('visual'):
   m=trimesh.load_mesh((OUT/'urdf'/v.find('geometry/mesh').get('filename')).resolve(),process=False);m.apply_transform(ts[l.get('name')]);c=[float(x) for x in v.find('material/color').get('rgba').split()][:3];items.append((m,c))
 render(items,OUT/'PREVIEW.png');print('render saved')
