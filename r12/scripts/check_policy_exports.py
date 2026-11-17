"""Compare checkpoint actor, TorchScript and standalone ONNX numerically."""
import json
from pathlib import Path
import numpy as np
import onnx
import onnxruntime as ort
import torch
root=Path(__file__).resolve().parents[1]
p=root/'models/r12/policy'
onnx.checker.check_model(str(p/'policy.onnx'))
network=torch.nn.Sequential(torch.nn.Linear(25,64),torch.nn.ELU(),torch.nn.Linear(64,64),torch.nn.ELU(),torch.nn.Linear(64,5))
state=torch.load(p/'model_599.pt',map_location='cpu',weights_only=False)['actor_state_dict']
network.load_state_dict({k.removeprefix('mlp.'):v for k,v in state.items() if k.startswith('mlp.')})
network.eval()
script=torch.jit.load(p/'policy.pt',map_location='cpu').eval()
session=ort.InferenceSession(str(p/'policy.onnx'),providers=['CPUExecutionProvider'])
rng=np.random.default_rng(131)
errors={'torchscript':0.,'onnx':0.}
for row in rng.uniform(-1,1,(256,25)).astype(np.float32):
    x=torch.from_numpy(row[None])
    with torch.no_grad():
        expected=network(x).numpy()
        actual=script(x).numpy()
    exported=session.run(None,{'obs':row[None]})[0]
    errors['torchscript']=max(errors['torchscript'],float(np.max(np.abs(actual-expected))))
    errors['onnx']=max(errors['onnx'],float(np.max(np.abs(exported-expected))))
report={'input_count':256,'input_shape':[1,25],'maximum_absolute_error':errors,
        'tolerance':1e-5,'passed':all(v<1e-5 for v in errors.values()),'onnx_self_contained':True}
(root/'reports/local/r12_export_parity.json').write_text(json.dumps(report,indent=2)+'\n')
print(report)
assert report['passed']
