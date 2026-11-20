"""Plot saved sweep results without rerunning physics."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    p = argparse.ArgumentParser()
    p.add_argument('directory', type=Path)
    args = p.parse_args()
    root = args.directory
    screen, mesh, verified = [read(root/f'{name}.jsonl') for name in ('screen', 'mesh', 'verified')]
    if (root/'final.jsonl').exists():
        verified = read(root/'final.jsonl')
    valid = lambda rows: [r for r in rows if 'error' not in r]
    screen, mesh, verified = map(valid, (screen, mesh, verified))
    families = list(dict.fromkeys(r['family'] for r in screen))
    amps = sorted({r['amplitude'] for r in screen})
    periods = sorted({r['period'] for r in screen})
    fig, axes = plt.subplots(4, 5, figsize=(18, 12), layout='constrained')
    limit = max(abs(r['late_speed']) for r in screen)*100
    for family, ax in zip(families, axes.flat):
        grid = np.array([[max(r['late_speed'] for r in screen if r['family'] == family
                             and r['amplitude'] == a and r['period'] == t)*100
                          for t in periods] for a in amps])
        im = ax.imshow(grid, origin='lower', aspect='auto', cmap='RdBu_r', vmin=-limit, vmax=limit)
        ax.set(title=family, xticks=range(len(periods)), xticklabels=periods,
               yticks=range(len(amps)), yticklabels=amps, xlabel='Period (s)', ylabel='Angle (deg)')
    fig.colorbar(im, ax=axes, label='Best phase: late +X speed (cm/s)')
    fig.suptitle('2,415-case screening: ellipsoid drag / neutral buoyancy (16 s)')
    fig.savefig(root/'screen_heatmaps.png', dpi=150)
    plt.close(fig)
    finalists = sorted([r for r in verified if r['neutral'] and r['dt'] == .001],
                       key=lambda r:r['late_speed'], reverse=True)
    fig, ax = plt.subplots(figsize=(11, 5), layout='constrained')
    labels = [f'{r["family"]} | {r["amplitude"]} deg | {r["period"]} s | {r["phase"]} deg' for r in finalists]
    ax.barh(labels[::-1], [r['late_speed']*100 for r in finalists][::-1], color='#147d92')
    ax.set(xlabel='Late-cycle +X speed (cm/s)', title='Mesh-surface verification: 64 s, 1 ms timestep, neutral buoyancy')
    fig.savefig(root/'verified_ranking.png', dpi=160)
    plt.close(fig)
    summary = {'screened': len(screen), 'mesh_evaluated': len(mesh), 'verified': len(verified),
               'finalists': finalists}
    if (root/'refined.jsonl').exists():
        summary['refined'] = len(read(root/'refined.jsonl'))
    if (root/'convergence.jsonl').exists():
        summary['convergence'] = read(root/'convergence.jsonl')
    (root/'summary.json').write_text(json.dumps(summary, indent=2))
    with (root/'final_ranking.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(finalists[0]))
        writer.writeheader()
        writer.writerows(finalists)


if __name__ == '__main__':
    main()
