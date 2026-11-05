"""Read memory and SHM provenance only for supervisor-owned process groups."""
import os
from pathlib import Path

def sample_owned(groups):
    rows=[]
    groups=set(groups)
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        pid=int(proc.name)
        try:
            group=os.getpgid(pid)
            if group not in groups:continue
            # Recheck after reads to avoid attributing a process that moved groups.
            memory={}
            for line in (proc/'smaps_rollup').read_text().splitlines():
                parts=line.split(); key=parts[0].rstrip(':')
                if key in ('Rss','Pss','Pss_Anon','Pss_File','Pss_Shmem','SwapPss'):memory[key]=int(parts[1])
            mappings=[]
            for line in (proc/'maps').read_text().splitlines():
                parts=line.split(maxsplit=5)
                if len(parts)==6 and parts[5].startswith('/dev/shm/'):
                    mappings.append({'path':parts[5],'inode':int(parts[4]),'device':parts[3]})
            if os.getpgid(pid)==group:
                rows.append({'pid':pid,'process_group':group,'comm':(proc/'comm').read_text().strip(),'memory_kib':memory,'shm_mappings':mappings})
        except (OSError,ValueError):continue
    return rows
