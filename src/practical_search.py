"""Conservative candidate screening using existing dynamics, not hardware certification."""
import json
from pathlib import Path
from src.search_motion import run_batch, rank, spec_of, control_for
from src.model import load_config
import yaml

def main():
    out=Path('output/practical_search_60')
    out.mkdir(parents=True,exist_ok=True)
    specs=[]
    for f in ['sine','tail','head','harmonic','asym25','asym75']:
        for a in [5,10,15,20,25,30,35,40,45]:
            for t in [1.5,2.,3.,4.,6.]:
                for phase in [-90,-60,-30]:
                    specs.append(dict(id=f'p{len(specs):04}',family=f,amplitude=a,period=t,phase=phase))
    def batch(name,jobs):
        path=out/f'{name}.jsonl'
        if path.exists():
            rows=[json.loads(x) for x in path.read_text().splitlines()]
            if len(rows)!=len(jobs): raise ValueError('Incomplete stage '+name)
            return rows
        return run_batch(jobs,path,4)
    def acceptable(r):
        return ('error' not in r and r['actual_max_deg']<=60 and
                r['tracking_rmse_deg']<=12 and r['saturation_fraction']<=.01 and r['late_speed']>0)
    screen=batch('screen',[(s,'ellipsoid',24.,.002,True) for s in specs])
    selected={r['id']:r for r in rank([r for r in screen if acceptable(r)])[:30]}
    for f in {s['family'] for s in specs}:
        for r in rank([r for r in screen if r['family']==f and acceptable(r)])[:5]: selected[r['id']]=r
    mesh=batch('mesh',[(spec_of(r),'mesh_surface',48.,.001,True) for r in selected.values()])
    winners=rank([r for r in mesh if acceptable(r)])[:5]
    if not winners: raise RuntimeError('No candidates met provisional limits')
    verified=batch('verified',[(spec_of(r),'mesh_surface',96.,.0005,True) for r in winners])
    winners=rank([r for r in verified if acceptable(r)])
    for i,r in enumerate(winners,1):
        c=load_config(); c['control']=control_for(r); c['physics']['neutral']=True
        c['simulation'].update(duration=96.,timestep=.0005)
        (out/f'candidate_{i:02}.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    (out/'summary.json').write_text(json.dumps(winners,indent=2))
    print(json.dumps(winners,indent=2),flush=True)

if __name__=='__main__': main()
