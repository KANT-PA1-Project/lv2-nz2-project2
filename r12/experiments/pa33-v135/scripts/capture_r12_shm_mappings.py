"""Read only /proc evidence for exact profile path of isolated diagnostic processes."""
import json,sys
from pathlib import Path
profile=str(Path(sys.argv[1]).resolve());output=Path(sys.argv[2]);rows=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  comm=(p/'comm').read_text().strip()
  if comm not in ('python','component_conta'):continue
  env=dict(x.split('=',1) for x in (p/'environ').read_bytes().decode().split('\0') if '=' in x)
  value=env.get('FASTDDS_DEFAULT_PROFILES_FILE')
  if not value or str(Path(value).resolve())!=profile:continue
  maps=[x for x in (p/'maps').read_text().splitlines() if '/dev/shm/fastdds_' in x];sizes={}
  for line in maps:
   name=line.split()[-1]
   try:sizes[name]=Path(name).stat().st_size
   except OSError:pass
  rows.append(dict(pid=int(p.name),cmd=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(),profile=value,shm_file_sizes=sizes,maps=maps))
 except (OSError,ValueError):pass
output.write_text(json.dumps(rows,indent=2)+'\n');print([(r['pid'],sorted(set(r['shm_file_sizes'].values()))) for r in rows])
