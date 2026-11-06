"""Execute actual runtime methods with a deterministic slow geometry checker."""
import ast
from pathlib import Path
import numpy as np
import pytest

@pytest.mark.parametrize('method',['final_clear','hold_safe'])
@pytest.mark.parametrize('elapsed,expected',[(.005,True),(.015,False)])
def test_geometry_cannot_outlive_sensor_evidence(method,elapsed,expected):
    path=Path(__file__).resolve().parents[1]/'scripts/sensor_runtime/live_oriented_path.py'
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and any(isinstance(x,ast.FunctionDef) and x.name=='final_clear' for x in n.body))
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('final_clear','hold_safe','_context_reason')]
    tick=[.140]
    def geometry(*args,**kwargs):tick[0]+=elapsed;return True
    namespace={'np':np,'motion_clear':geometry,'uncertainty_footprint':lambda f,*args:f}
    extracted=ast.ClassDef(name='Boundary',bases=[],keywords=[],body=methods,decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[extracted],type_ignores=[])),str(path),'exec'),namespace)
    obj=namespace['Boundary']();obj.clock=lambda:tick[0];obj.max_sensor_age=.15;obj.max_map_age=1.
    obj.sensor={'valid':True,'received':0.,'sim_stamp_ns':1,'transport_settled':True,'pose':[0,0,0]}
    obj.map={'received':0.,'stamp_ns':1,'origin':[0,0],'occupied':np.zeros((2,2),bool),'resolution':1.}
    obj.plan={};obj.footprint=[[0,0],[1,0],[0,1]];obj.position_margin=0.;obj.yaw_error=0.
    obj.reference_inputs=lambda:dict(xy=[0,0],yaw=0,footprint=obj.footprint,occupied=obj.map['occupied'],resolution=1.)
    result=obj.final_clear([0,0,0]) if method=='final_clear' else obj.hold_safe()
    assert result is expected
