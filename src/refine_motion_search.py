"""Refine the broad sweep in the actual mesh drag model and export finalists."""
import argparse
from itertools import product
from pathlib import Path

import yaml

from .model import load_config
from .motion_patterns import patterns
from .report_motion_search import read
from .search_motion import control_for, rank, run_batch, spec_of


def main():
    p = argparse.ArgumentParser()
    p.add_argument('directory', type=Path)
    p.add_argument('--workers', type=int, default=4)
    args = p.parse_args()
    root = args.directory
    screen, mesh = [read(root/f'{name}.jsonl') for name in ('screen', 'mesh')]
    seeds = []
    for row in rank(mesh):
        if row['family'] not in {r['family'] for r in seeds}:
            seeds.append(row)
        if len(seeds) == 3:
            break
    specs = []
    for seed in seeds:
        phases = [seed['phase']] if seed['family'] in patterns() else [seed['phase']+d for d in [-15, 0, 15]]
        for amplitude, period, phase in product(
                sorted({max(5, min(90, seed['amplitude']+d)) for d in [-15, 0, 15]}),
                [seed['period']*k for k in [.75, 1., 1.25]], phases):
            specs.append(dict(id=f'r{len(specs):03}', family=seed['family'], amplitude=amplitude,
                              period=period, phase=phase))
    existing = {r['id'] for r in mesh}
    specs.extend(spec_of(r) for r in rank([r for r in screen if r['amplitude'] <= 20])[:12]
                 if r['id'] not in existing)
    specs.extend([dict(id='baseline', family='sine', amplitude=30, period=2., phase=-45),
                  dict(id='stationary', family='sine', amplitude=0, period=2., phase=-45)])
    def batch(name, jobs):
        path = root/f'{name}.jsonl'
        if path.exists():
            rows = read(path)
            if len(rows) != len(jobs):
                raise ValueError(f'Incomplete stage: {path}')
            return rows
        return run_batch(jobs, path, args.workers)
    refined = batch('refined', [(s, 'mesh_surface', 32., .002, True) for s in specs])
    combined = rank(mesh + refined)
    finalists = {r['id']: r for r in combined[:6]}
    small = rank([r for r in combined if r['amplitude'] <= 20])[:2]
    finalists.update({r['id']: r for r in small})
    finalists.update({r['id']: r for r in refined if r['id'] in ('baseline', 'stationary')})
    final = batch('final', [(spec_of(r), 'mesh_surface', 64., dt, neutral)
                            for r in finalists.values()
                            for dt, neutral in [(.002, True), (.001, True), (.002, False)]])
    winners = rank([r for r in final if r['neutral'] and r['dt'] == .001])
    batch('convergence', [(spec_of(winners[0]), 'mesh_surface', 64., .0005, True)])
    base = load_config()
    base['physics']['neutral'] = True
    base['simulation'].update(duration=64., timestep=.001)
    cases = []
    chosen = winners[:3] + [r for r in winners if r['amplitude'] <= 20][:1]
    chosen += [r for r in winners if r['id'] == 'baseline']
    for i, row in enumerate(chosen, 1):
        cfg = load_config()
        cfg['physics']['neutral'] = True
        cfg['simulation'].update(duration=64., timestep=.001)
        cfg['control'] = control_for(row)
        (root/f'final_{i:02}.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))
        cases.append(dict(name=f'{row["id"]}_{row["family"]}_{row["amplitude"]}deg_{row["period"]}s',
                          control=cfg['control']))
    base['comparison'] = dict(cases=cases)
    (root/'final_comparison.yaml').write_text(yaml.safe_dump(base, sort_keys=False, allow_unicode=True))
    for row in winners:
        print(row, flush=True)


if __name__ == '__main__':
    main()
