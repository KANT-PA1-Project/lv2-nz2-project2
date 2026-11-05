from pathlib import Path
import os,time

def snapshot(pgid):
 rows=[]
 for p in Path('/proc').glob('[0-9]*'):
  try:
   a=(p/'stat').read_text().rsplit(')',1)[1].split()
   if int(a[2])!=pgid:continue
   stat=(p/'status').read_text();rss=int(next(l for l in stat.splitlines()if l.startswith('VmRSS:')).split()[1]);rows.append(dict(pid=int(p.name),ppid=int(a[1]),state=a[0],cpu_ticks=int(a[11])+int(a[12]),rss_kib=rss,threads=int(a[17]),affinity=sorted(os.sched_getaffinity(int(p.name)))))
  except(OSError,StopIteration,ProcessLookupError):pass
 mem=int(next(l for l in Path('/proc/meminfo').read_text().splitlines()if l.startswith('MemAvailable:')).split()[1])
 return dict(monotonic_ns=time.monotonic_ns(),processes=rows,rss_sum_kib=sum(r['rss_kib']for r in rows),mem_available_kib=mem,clock_ticks_per_second=os.sysconf('SC_CLK_TCK'))
