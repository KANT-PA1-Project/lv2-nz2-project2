"""Inject failure only into owned diagnostic children, not simulator processes."""
import json,os,signal,subprocess,sys,tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from arm_practice.shared_frame_prototype import SharedFrameSlot
if '--child' in sys.argv:
    mode=sys.argv[-2];directory=sys.argv[-1]
    with SharedFrameSlot(capacity=4096,directory=directory) as slot:
        print(json.dumps(slot.publish({'pixels':np.arange(32,dtype=np.uint8)})),flush=True)
        if mode=='exception':raise RuntimeError('injected producer exception')
        if not sys.stdin.readline():raise ConnectionError('injected reader disconnect')
    sys.exit(0)
out=Path(__file__).resolve().parents[3]/'outputs/r12';results=[]
with SharedFrameSlot(capacity=4096) as unrelated:
    sentinel=unrelated.publish({'pixels':np.arange(32,dtype=np.uint8)})
    for mode in ('exception','disconnect','sigterm','sigkill'):
        directory=tempfile.TemporaryDirectory(prefix='r12-frame-',dir='/dev/shm');p=None
        try:
            p=subprocess.Popen([sys.executable,__file__,'--child',mode,directory.name],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            descriptor=json.loads(p.stdout.readline());assert descriptor['path']==str(Path(directory.name)/'slot')
            if mode=='disconnect':p.stdin.close()
            elif mode=='sigterm':p.terminate()
            elif mode=='sigkill':p.kill()
            p.wait(timeout=5)
            existed_before_cleanup=Path(descriptor['path']).exists()
        finally:
            if p is not None and p.poll() is None:p.kill();p.wait(timeout=5)
            directory.cleanup()
        assert not Path(directory.name).exists() and unrelated.path.exists()
        results.append({'mode':mode,'child_exit':p.returncode,'slot_existed_after_child_exit':existed_before_cleanup,'supervisor_directory_removed':True,'unrelated_slot_preserved':True})
report={'scope':'Owned synthetic subprocess lifecycle injection; no GUI run or training','cases':results,'all_passed':len(results)==4,'remaining_test_children':False,'runtime_launcher_prepared':'run_r12_stationary_v344.py','gui_executed':False}
(out/'v344_lifecycle_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
