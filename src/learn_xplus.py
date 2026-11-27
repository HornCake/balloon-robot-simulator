"""Learn a head-to-tail wave by constrained local policy search."""
import argparse
from pathlib import Path

import yaml

from .model import load_config
from .search_motion import control_for, run_batch


def feasible(row):
    return ('error' not in row and row['actual_max_deg'] <= 60
            and row['tracking_rmse_deg'] <= 12 and row['saturation_fraction'] <= .01
            and row['late_speed'] > 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('output/learn_xplus'))
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    baseline = dict(id='previous', family='head', amplitude=40., period=1.875, phase=-72., taper=.25, phase_curve=0.)
    fields = ('amplitude', 'period', 'phase', 'taper', 'phase_curve')
    best = baseline
    scores = {}
    specs = {}
    # Each row changes amplitude, period, phase, taper and the phase increment
    # between successive joints. Scale the neighbourhood around the best gait.
    moves = [(0, 0, 0, 0, 0), (0, 0, 0, -.05, -12), (0, 0, 0, 0, 15),
             (0, 0, 0, 0, -15), (0, 0, 12, 0, -24), (0, 0, -12, 0, 24),
             (3, 0, 0, .05, 0), (0, -.125, -8, .025, 0), (2, 0, -6, 0, 0)]
    for generation, scale in enumerate((1., .5, .25)):
        jobs = []
        for move in moves:
            key = tuple(round(best[k] + delta * scale, 6) for k, delta in zip(fields, move))
            amplitude, _, _, taper, _ = key
            if key in specs or amplitude > 45 or not 0 <= taper <= 1/3:
                continue
            spec = dict(best, **dict(zip(fields, key)))
            spec['id'] = f'g{generation}_{len(jobs)}'
            specs[key] = spec
            jobs.append((spec, 'mesh_surface', 12., .001, True, 60.))
        for row in run_batch(jobs, args.output / f'generation_{generation}.jsonl', args.workers):
            if feasible(row):
                scores[tuple(row[k] for k in fields)] = row
        if scores:
            best = specs[max(scores, key=lambda key: scores[key]['dx'])]
    finalists = [baseline]
    # Leave tracking headroom for the longer, finer-step validation.
    eligible = [key for key, row in scores.items() if row['tracking_rmse_deg'] <= 11.8
                and key != tuple(baseline[k] for k in fields)]
    if eligible:
        finalists.append(specs[max(eligible, key=lambda key: scores[key]['dx'])])
    rows = run_batch([(s, 'mesh_surface', 96., .0005, True, 60.) for s in finalists],
                     args.output / 'validation.jsonl', args.workers)
    winner = max(filter(feasible, rows), key=lambda r: r['dx'])
    cfg = load_config()
    cfg['joint']['range_deg'] = 60
    cfg['physics'].update(neutral=True, fluid_model='mesh_surface')
    cfg['simulation'].update(duration=96., timestep=.0005)
    cfg['control'] = control_for(winner)
    (args.output / 'winner.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))


if __name__ == '__main__':
    main()
