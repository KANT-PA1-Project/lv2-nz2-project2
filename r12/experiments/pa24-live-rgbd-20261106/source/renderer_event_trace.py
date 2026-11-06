"""Read-only trace helper. Does not enable settings or grant motion authority."""
import json,time,traceback
from pathlib import Path
KEYS=('/rtx/post/aa/op','/rtx-transient/post/aa/limitedOps','/rtx-defaults/post/aa/op','/rtx/rendermode','/rtx/post/dlss/execMode')
class RendererTrace:
 def __init__(self,out,settings,subscribe):
  self.path=Path(out)/'renderer_events.jsonl';self.settings=settings;self.phase='attach';self.subscriptions=[]
  for key in KEYS:
   self.subscriptions.append(subscribe(key,lambda *args,key=key:self.event('setting_change',key=key,stack=traceback.format_stack(limit=12))))
  self.event('attached')
 def event(self,kind,**extra):
  record=dict(monotonic_ns=time.monotonic_ns(),phase=self.phase,kind=kind,settings={k:self.settings.get(k)for k in KEYS},**extra)
  with self.path.open('a')as f:f.write(json.dumps(record,default=str)+'\n')
 def mark(self,label,stage=None,step=None):
  self.phase=label;attrs=[]
  if stage is not None:
   for prim in stage.Traverse():
    if prim.GetTypeName()in ('RenderSettings','RenderProduct') or str(prim.GetPath()).startswith('/Render'):
     relevant={a.GetName():str(a.Get()) for a in prim.GetAttributes()if any(x in a.GetName().lower()for x in ('aa:','rendermode','dlss','renderersettings'))}
     if relevant:attrs.append(dict(path=str(prim.GetPath()),type=prim.GetTypeName(),attributes=relevant))
  self.event('checkpoint',physics_step=step,stage_render_attributes=attrs)
