"""Reproducible +X gait search; run with python -m src.search_motion."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from copy import deepcopy
import csv
import json
from pathlib import Path
import time

import mujoco
import numpy as np
import yaml

from .controller import targets
from .model import load_config
from .motion_patterns import apply_pattern, patterns
from .simulation import Simulation


def search_step(sim):
    # reset()/the previous step already refreshed controls and external forces.
    # Avoid recomputing identical mesh pressures at the start of every step.
    mujoco.mj_step(sim.model, sim.data)
    sim.refresh()
    if not np.all(np.isfinite(sim.data.qpos)) or np.any(sim.data.warning.number):
        raise RuntimeError('MuJoCo numerical warning; reduce timestep or controller gains')


def control_for(spec):
    c = deepcopy(load_config()['control'])
    c.update(period=spec['period'], motion_scale=1.)
    a = spec['amplitude']
    family = spec['family']
    if family in patterns():
        apply_pattern(c, family)
        sample = np.array([targets(t, c) for t in np.linspace(0, spec['period'], 401)])
        c['motion_scale'] = a / np.rad2deg(np.max(np.abs(sample)))
    elif family in ('sine', 'asym25', 'asym75'):
        c.update(mode='traveling' if family == 'sine' else 'asymmetric',
                 amplitude_deg=a, phase_step_deg=spec['phase'],
                 bend_fraction=.25 if family == 'asym25' else .75)
    else:
        waves = {'triangle': '1-4*max(u(n,t)-0.5,0)-4*max(0.5-u(n,t),0)',
                 'harmonic': '(sin(p(n,t))+0.3*sin(3*p(n,t)))/0.93',
                 'tail': 'n/4*sin(p(n,t))',
                 'head': '(5-n)/4*sin(p(n,t))'}
        c.update(mode='expressions', expression_period=spec['period'],
                 expressions=[f'A = {a}', f'T = {spec["period"]}',
                              f'd = {spec["phase"]}/360',
                              'u(n,t) = (t/T-(n-1)*d)%1',
                              'p(n,t) = 2*pi*u(n,t)',
                              f'x(n,t) = A*({waves[family]})'])
    return c


def evaluate(job):
    spec, fluid, duration, dt, neutral = job
    cfg = load_config()
    cfg['control'] = control_for(spec)
    cfg['physics'].update(fluid_model=fluid, neutral=neutral)
    cfg['simulation'].update(duration=duration, timestep=dt)
    start = time.monotonic()
    try:
        s = Simulation(cfg)
        # Measure the final whole cycles after at least half the run has elapsed.
        cycles = max(1, int(duration / (2 * spec['period'])))
        cut = duration - cycles * spec['period']
        initial_late = None
        err, saturation, count, actual = 0., 0, 0, 0.
        for step in range(round(duration / dt)):
            search_step(s)
            if initial_late is None and s.data.time >= cut - dt/2:
                initial_late = (s.data.time, s.center_of_mass.copy())
            if step % 10 == 0:
                err += float(np.mean((s.data.ctrl - s.data.qpos[7:])**2))
                saturation += int(np.any(np.abs(s.data.actuator_force) >= .1799))
                actual = max(actual, float(np.max(np.abs(s.data.qpos[7:]))))
                count += 1
        late_t, late_pos = initial_late
        dx, dy, dz = map(float, s.displacement)
        return dict(**spec, fluid=fluid, duration=duration, dt=dt, neutral=neutral,
                    dx=dx, dy=dy, dz=dz, speed=dx/s.data.time,
                    late_speed=float((s.center_of_mass[0]-late_pos[0])/(s.data.time-late_t)),
                    tracking_rmse_deg=float(np.rad2deg(np.sqrt(err/count))),
                    saturation_fraction=saturation/count, actual_max_deg=float(np.rad2deg(actual)),
                    wall_seconds=time.monotonic()-start)
    except (ValueError, RuntimeError) as e:
        return dict(**spec, fluid=fluid, duration=duration, dt=dt, neutral=neutral, error=str(e))


def candidates():
    result = []
    for family in list(patterns()) + ['sine', 'asym25', 'asym75', 'triangle', 'harmonic', 'tail', 'head']:
        phases = [0] if family in patterns() else [-120, -90, -60, -30, 30, 60, 90, 120]
        for amplitude in [10, 20, 30, 45, 60, 75, 90]:
            for period in [.5, 1., 2., 4., 8.]:
                for phase in phases:
                    result.append(dict(family=family, amplitude=amplitude, period=period, phase=phase))
    return [dict(id=f'g{i:04}', **s) for i, s in enumerate(result)]


def run_batch(jobs, path, workers):
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool, path.open('w') as f:
        for row in pool.map(evaluate, jobs, chunksize=4):
            results.append(row)
            f.write(json.dumps(row)+'\n')
            f.flush()
            if len(results) % 100 == 0 or len(results) == len(jobs):
                print(f'{path.stem}: {len(results)}/{len(jobs)}', flush=True)
    return results


def rank(rows):
    return sorted([r for r in rows if 'error' not in r], key=lambda r: r['late_speed'], reverse=True)


def spec_of(row):
    return {k: row[k] for k in ('id', 'family', 'amplitude', 'period', 'phase')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=Path('output/motion_search'))
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    specs = candidates()
    def batch(name, jobs):
        path = args.output / f'{name}.jsonl'
        if path.exists():
            rows = [json.loads(x) for x in path.read_text().splitlines()]
            if len(rows) == len(jobs):
                return rows
            raise ValueError(f'Incomplete stage: {path}; move it before restarting')
        return run_batch(jobs, path, args.workers)
    screen = batch('screen', [(s, 'ellipsoid', 16., .002, True) for s in specs])
    selected = {r['id']: spec_of(r) for r in rank(screen)[:60]}
    # Retain each waveform and each period even if the surrogate ranks it poorly.
    for family in {s['family'] for s in specs}:
        for period in [.5, 1., 2., 4., 8.]:
            for r in rank([r for r in screen if r['family'] == family and r['period'] == period])[:2]:
                selected[r['id']] = spec_of(r)
    mesh = batch('mesh', [(s, 'mesh_surface', 32., .002, True) for s in selected.values()])
    top = rank(mesh)[:8]
    verified = batch('verified', [(spec_of(r), 'mesh_surface', 64., dt, neutral)
                                 for r in top for dt, neutral in [(.002, True), (.001, True), (.002, False)]])
    for name, rows in [('screen', screen), ('mesh', mesh), ('verified', verified)]:
        keys = list(dict.fromkeys(k for r in rows for k in r))
        with (args.output/f'{name}.csv').open('w') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rank(rows))
    finalists = rank([r for r in verified if r['neutral'] and r['dt'] == .001])
    cases = []
    for i, row in enumerate(finalists):
        c = load_config()
        c['control'] = control_for(row)
        c['physics']['neutral'] = True
        c['simulation'].update(duration=64., timestep=.001)
        (args.output/f'best_{i+1:02}.yaml').write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
        cases.append(dict(name=f'{i+1}_{row["id"]}_{row["family"]}', control=c['control']))
    c = load_config()
    c['physics']['neutral'] = True
    c['simulation'].update(duration=64., timestep=.001)
    c['comparison'] = {'cases': cases[:5]}
    (args.output/'comparison.yaml').write_text(yaml.safe_dump(c, allow_unicode=True, sort_keys=False))
    print(json.dumps(finalists, indent=2), flush=True)


if __name__ == '__main__':
    main()
