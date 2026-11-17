import sys,types,numpy as np,pytest
from arm_practice.tag_detection import detect_tags

def run(monkeypatch,items):
 fake=types.SimpleNamespace(INTER_CUBIC=1,resize=lambda gray,none,fx,fy,interpolation:np.zeros((int(10*fy),int(10*fx))))
 monkeypatch.setitem(sys.modules,'cv2',fake)
 class Detector:
  def __init__(self):self.calls=[]
  def detectMarkers(self,image):
   scale=image.shape[0]//10;self.calls.append(scale);lst=items.get(scale,[])
   return [np.array([[[x,0],[x+10,0],[x+10,10],[x,10]]],np.float32)*scale for ident,x in lst],np.array([[ident]for ident,x in lst])if lst else None,[]
 d=Detector();r=detect_tags(np.zeros((10,10)),d);return r,d.calls

def test_no_fallback_two_agreed_faces(monkeypatch):
 r,c=run(monkeypatch,{1:[(0,0),(0,100)],2:[(0,0),(0,100)]});assert c==[1,2];assert len(r)==2

def test_fallback_preserves_two_scale_rule(monkeypatch):
 r,c=run(monkeypatch,{1:[(0,0)],2:[(0,0),(0,100)],3:[(0,0),(0,100)]});assert c==[1,2,3];assert len(r)==2;assert r[1]['agreeing_scales']==[2,3]

def test_only_scale3_rejected(monkeypatch):
 r,c=run(monkeypatch,{3:[(0,0)]});assert r==[];assert c==[1,2,3]

def test_conflicting_identity_rejected(monkeypatch):
 r,c=run(monkeypatch,{1:[(0,0)],2:[(0,0)],3:[(1,0)]});assert r==[]

def test_other_ids_not_two_faces(monkeypatch):
 r,c=run(monkeypatch,{1:[(0,0),(1,100)],2:[(0,0),(1,100)]});assert c==[1,2,3]

def test_empty_is_bounded(monkeypatch):
 r,c=run(monkeypatch,{});assert r==[];assert c==[1,2,3]
