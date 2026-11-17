"""Read-only conservative admission for the bounded R12 profiling experiment."""
import json, time, shutil
from pathlib import Path

def snapshot():
    mem={line.split(':')[0]:int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()}
    pressure={}
    for name in ('memory','io'):
        pressure[name]={line.split()[0]:{k:float(v) for k,v in (x.split('=') for x in line.split()[1:])} for line in Path('/proc/pressure',name).read_text().splitlines()}
    return {'monotonic_ns':time.monotonic_ns(),'mem_available_bytes':mem['MemAvailable']*1024,'swap_free_bytes':mem['SwapFree']*1024,'pressure':pressure,'shm_free_bytes':shutil.disk_usage('/dev/shm').free,'disk_free_bytes':shutil.disk_usage(Path(__file__).resolve().parents[1]).free}

def check(output, minimum_available_gib=8):
    samples=[]
    for i in range(3):
        if i:time.sleep(1)
        samples.append(snapshot())
    reasons=[]
    for s in samples:
        if s['mem_available_bytes']<minimum_available_gib*1024**3:reasons.append(f'available RAM below operational {minimum_available_gib} GiB reserve')
        if s['pressure']['memory']['full']['avg10']>1:reasons.append('memory full-stall avg10 above 1 percent')
        if s['pressure']['io']['full']['avg10']>5:reasons.append('IO full-stall avg10 above 5 percent')
        if s['shm_free_bytes']<512*1024**2:reasons.append('shared-memory reserve below 512 MiB')
        if s['disk_free_bytes']<512*1024**2:reasons.append('disk reserve below 512 MiB')
    result={'minimum_available_gib':minimum_available_gib,'samples':samples,'admitted':not reasons,'reasons':sorted(set(reasons)),'scope':'Operational diagnostic guard, not robot acceptance criteria or a proven simulator memory requirement. Passing does not guarantee resource sufficiency.'}
    Path(output).write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    import sys
    result=check(sys.argv[1]);print(json.dumps(result));sys.exit(0 if result['admitted'] else 2)
