"""Q1 practice navigation itinerary; no fabricated pickup/perception evidence."""
from pathlib import Path
import sys,json,itertools,argparse,numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.mission import plan
from arm_practice.navigation import Grid

parser=argparse.ArgumentParser();parser.add_argument('--seed',type=int);args=parser.parse_args()
if args.seed is None:fixture=json.loads((root/'configs/mission/practice.json').read_text())
else:
    from arm_practice.mission_random import generate
    fixture=generate(args.seed)
    (root/f'configs/mission/random_{args.seed}.json').write_text(json.dumps(fixture,indent=2))
source_by_id={p['id']:p.get('source',f"S{p['id']//2+1}") for p in fixture['pucks']}
allocation=plan(fixture['sequence'],fixture['barcode'],fixture['pucks'])
route=json.loads((root/'models/navigation/routes.json').read_text());points=route['points'];grid=Grid(root/'models/navigation/maze_grid.npz')
names=list(points);distance={};paths={}
for i,a in enumerate(names):
    for b in names[i:]:
        path=grid.path(points[a],points[b]);length=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
        distance[a,b]=distance[b,a]=length;paths[a,b]=path.tolist();paths[b,a]=path[::-1].tolist()
jobs=[]
for location in allocation['locations']:
    for target in location['targets']:
        pid=target['id'];jobs.append({'puck_id':pid,'source':source_by_id[pid],'destination':location['location']})
# Practice source assignment is explicit and synthetic: IDs 2k,2k+1 on S(k+1).
sources=[f'S{i}' for i in range(1,7)]
survey=min(itertools.permutations(sources),key=lambda p:sum(distance[a,b] for a,b in zip(('START',)+p,p)))
start=survey[-1];n=len(jobs);cost={};previous={}
for i,j in enumerate(jobs):cost[1<<i,i]=distance[start,j['source']]+distance[j['source'],j['destination']]
for mask in range(1,1<<n):
    for i in range(n):
        if (mask,i) not in cost:continue
        for j in range(n):
            if mask&(1<<j):continue
            key=(mask|(1<<j),j);candidate=cost[mask,i]+distance[jobs[i]['destination'],jobs[j]['source']]+distance[jobs[j]['source'],jobs[j]['destination']]
            if candidate<cost.get(key,float('inf')):cost[key]=candidate;previous[key]=(mask,i)
def verify_tail(start):
    options=itertools.permutations(['L1','L2','L3'])
    p=min(options,key=lambda p:sum(distance[a,b] for a,b in zip((start,)+p,p+('START',))))
    return p,sum(distance[a,b] for a,b in zip((start,)+p,p+('START',)))
mask=(1<<n)-1;last=min(range(n),key=lambda i:cost[mask,i]+verify_tail(jobs[i]['destination'])[1]);key=(mask,last);order=[]
while True:
    order.append(key[1])
    if key not in previous:break
    key=previous[key]
order.reverse();verification,_=verify_tail(jobs[last]['destination'])
steps=[{'station':s,'phase':'inventory_approach'} for s in survey]
for i in order:
    j=jobs[i]
    steps.extend([{'station':j['source'],'phase':'pickup_approach','puck_id':j['puck_id']},{'station':j['destination'],'phase':'placement_approach','puck_id':j['puck_id']}])
steps.extend({'station':s,'phase':'verification_approach'} for s in verification);steps.append({'station':'START','phase':'return'})
length=sum(distance[a,b] for a,b in zip(['START']+[s['station'] for s in steps],[s['station'] for s in steps]))
output={'fixture':fixture['description'],'scope':'Navigation-only synthetic practice itinerary. Arrival is NOT pickup, placement, signal recognition, or visual re-verification. No payload is attached.','allocation':allocation,'points':points,'steps':steps,'planned_grid_distance_m':length,'job_order':order,'all_pair_paths_connected':len(paths),'planner':'A* collision-grid distances; shortest survey permutation, then exact single-puck transfer-order DP and verification order.'}
(root/('models/navigation/q1_practice_route.json' if args.seed is None else f'models/navigation/random_{args.seed}_route.json')).write_text(json.dumps(output,ensure_ascii=False,indent=2));print(json.dumps({'steps':len(steps),'planned_distance_m':length,'allocation_counts':allocation['counts']},ensure_ascii=False))
