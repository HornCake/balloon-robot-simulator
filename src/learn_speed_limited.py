"""Select long-period gaits under per-joint peak angular velocity limits."""
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path

import numpy as np
import yaml

from .learn_xplus import feasible
from .model import ROOT, load_config
from .search_motion import control_for, search_step
from .simulation import Simulation

OUTPUT = ROOT / 'output' / 'speed_limited'
REFERENCE = ROOT / 'config' / 'xplus_learned.yaml'
LIMITS = (.75, .50)


def configure(spec):
    cfg = load_config(REFERENCE)
    if 'config' in spec:
        return load_config(ROOT / 'config' / spec['config'])
    ctrl = control_for(dict(family='head', **spec))
    ctrl['expressions'][-1] += ' * (0.5-0.5*cos(pi*min(1,t/T)))'
    cfg['control'] = ctrl
    return cfg


def measure(job):
    spec, duration, dt = job
    cfg = configure(spec)
    cfg['simulation'].update(duration=duration, timestep=dt)
    sim = Simulation(cfg)
    period = cfg['control']['period']
    peak = np.zeros(4)
    running = np.zeros(4)
    actual = error = saturated = 0.
    late_start = duration - max(1, int(duration/(2*period))) * period
    steps = round(duration/dt)
    torque_limit = cfg['joint']['torque_limit']
    for step in range(1, steps+1):
        search_step(sim)
        speeds = np.abs(sim.data.qvel[6:])
        np.maximum(peak, speeds, out=peak)
        if step*dt >= period:
            np.maximum(running, speeds, out=running)
        angles = sim.data.qpos[7:]
        actual = max(actual, np.abs(angles).max())
        error += np.square(sim.data.ctrl - angles).mean()
        saturated += np.any(np.abs(sim.data.actuator_force) >= torque_limit - .0001)
        if step == round(late_start/dt):
            late_x = sim.displacement[0]
    dx, dy, dz = sim.displacement
    return dict(spec=spec, duration=duration, dt=dt, peak=np.rad2deg(peak).tolist(),
                running=np.rad2deg(running).tolist(), dx=dx, dy=dy, dz=dz,
                late_speed=(dx-late_x)/(duration-late_start),
                actual_max_deg=np.rad2deg(actual), tracking_rmse_deg=np.rad2deg(np.sqrt(error/steps)),
                saturation_fraction=saturated/steps)


def ratio(row, reference):
    return max(np.max(np.divide(row[key], reference[key])) for key in ('peak', 'running'))


def batch(pool, name, specs, duration, dt):
    rows = list(pool.map(measure, [(spec, duration, dt) for spec in specs]))
    (OUTPUT / f'{name}.json').write_text(json.dumps(rows, indent=2))
    print(name, len(rows), flush=True)
    return rows


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    references = [dict(config=name) for name in ('xplus_learned.yaml', 'xplus_long_period.yaml')]
    with ProcessPoolExecutor(6) as pool:
        baseline, _ = batch(pool, 'references', references, 96., .0005)
        coarse = [dict(period=p, amplitude=a, taper=t, phase=d, phase_curve=c)
                  for p in (2.5, 3., 3.5, 3.75)
                  for a,t,d,c in [(42,.24,-72,-12),(45,.12,-54,-33),(42,.18,-66,-24),
                                  (38,.25,-56,-32),(32,.20,-80,4),(24,.24,-65,-16)]]
        rows = batch(pool, 'screen', coarse, 15., .001)
        adjustments = []
        for limit in LIMITS:
            for row in rows:
                spec = row['spec']
                factor = limit * .96 / ratio(row, baseline)
                adjustments.append(dict(spec, amplitude=round(min(45., spec['amplitude']*factor), 2)))
        adjusted = list({tuple(s.items()):s for s in adjustments}.values())
        rows += batch(pool, 'adjusted', adjusted, 15., .001)
        local = []
        for limit in LIMITS:
            seeds = sorted((r for r in rows if feasible(r) and ratio(r, baseline)<=limit),
                           key=lambda r:-r['dx'])[:2]
            for row in seeds:
                s = row['spec']
                for dp, dc, taper in [(8,-12,-.035),(-8,12,.035),(4,8,0),(-4,-8,0)]:
                    local.append(dict(s, phase=s['phase']+dp, phase_curve=s['phase_curve']+dc,
                                      taper=round(s['taper']+taper,3)))
        rows += batch(pool, 'local', local, 15., .001)
        finalists = []
        for limit in LIMITS:
            ordered = sorted((r for r in rows if feasible(r) and ratio(r, baseline)<=limit),
                             key=lambda r:-r['dx'])
            for row in ordered[:3]:
                if row['spec'] not in finalists:
                    finalists.append(row['spec'])
        verified = batch(pool, 'validation', finalists, 96., .0005)
        for limit in LIMITS:
            best = max((r for r in verified if feasible(r) and ratio(r, baseline)<=limit),
                       key=lambda r:r['dx'])
            path = ROOT / 'config' / f'xplus_speed_{round(100*limit)}.yaml'
            path.write_text(yaml.safe_dump(configure(best['spec']), sort_keys=False))
            print(limit, best, flush=True)


if __name__ == '__main__':
    main()
