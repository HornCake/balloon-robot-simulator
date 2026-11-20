"""Run with uv run python -m src.main (macOS viewer: uv run mjpython -m src.main)."""
import argparse
import math
from pathlib import Path
import time
import yaml
from .model import ROOT, load_config, build_model, validate_config
from .comparison import Comparison, ComparisonLogger, cases_from_config
from .simulation import Simulation
from .logger import Logger
from .visualization import view, plot, plot_comparison
from .motion_patterns import patterns, apply_pattern, clear_pattern


def nonnegative_seconds(value):
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0:
        raise argparse.ArgumentTypeError('Seconds must be finite and nonnegative')
    return seconds


def configure(c, args):
    if args.duration is not None:
        c['simulation']['duration'] = args.duration
    for flag, field in [(args.no_buoyancy, 'buoyancy'), (args.no_fluid, 'fluid')]:
        if flag:
            c['physics'][field] = False
    if args.neutral:
        c['physics']['neutral'] = True
    if args.mode:
        clear_pattern(c['control'])
        c['control']['mode'] = args.mode
    if args.pattern:
        apply_pattern(c['control'], args.pattern)
    if args.motion_scale is not None:
        c['control']['motion_scale'] = args.motion_scale
    if args.period is not None:
        c['control']['period'] = args.period
    if args.phase_step_deg is not None:
        c['control']['phase_step_deg'] = args.phase_step_deg
    if args.fluid_model:
        c['physics']['fluid_model'] = args.fluid_model
    if args.mesh:
        c['balloon']['mesh_file'] = str(args.mesh.expanduser().resolve())
    for argument, field in [('mesh_scale', 'mesh_scale'), ('mesh_rotation', 'mesh_rotation_deg'),
                            ('dimensions', 'dimensions'), ('mesh_center', 'mesh_center')]:
        value = getattr(args, argument)
        if value is not None:
            c['balloon'][field] = value
    if args.fit_mesh:
        c['balloon']['mesh_fit_dimensions'] = True
    return validate_config(c)


def main():
    parser = argparse.ArgumentParser(description='Five connected helium balloons')
    parser.add_argument('--config', type=Path, default=ROOT/'config/default.yaml')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--realtime', action='store_true', help='Pace headless or viewer execution to real time')
    parser.add_argument('--windowed', action='store_true', help='Use the original window size instead of starting maximized')
    startup = parser.add_mutually_exclusive_group()
    startup.add_argument('--start-paused', action='store_true', help='Open the viewer paused; Space starts the simulation')
    startup.add_argument('--start-delay', type=nonnegative_seconds, default=0, metavar='SECONDS',
                         help='Hold the initial state for this many wall-clock seconds before starting')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--no-buoyancy', action='store_true')
    parser.add_argument('--no-fluid', action='store_true')
    parser.add_argument('--neutral', action='store_true')
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument('--mode', choices=['fixed','sine','traveling','asymmetric'])
    motion.add_argument('--pattern', choices=list(patterns()), help='Run a registered joint motion pattern')
    parser.add_argument('--list-patterns', action='store_true', help='List registered motion patterns and exit')
    parser.add_argument('--motion-scale', type=float,
                        help='Multiply all joint angle commands by this nonnegative factor before the +/-90 degree limit')
    parser.add_argument('--period', type=float, metavar='SECONDS',
                        help='Common duration of one complete motion cycle (default config: 2 seconds)')
    parser.add_argument('--phase-step-deg', type=float, help='Inter-joint phase delay in degrees; default -45 drives the default traveling gait toward +X')
    parser.add_argument('--camera', choices=['overview','side','follow','top'], default='overview')
    parser.add_argument('--projection', choices=['perspective','orthographic'],
                        help='Override projection; top defaults to orthographic, others to perspective')
    parser.add_argument('--x-direction', choices=['left','right'], default='right',
                        help='Initial screen direction of world +X (does not change physics coordinates)')
    parser.add_argument('--focus-case', type=int, metavar='N', help='Show only comparison case N (1-based)')
    parser.add_argument('--side-all', action='store_true', help='Show all comparison cases together in true side view (A toggles this)')
    parser.add_argument('--output', type=Path, default=ROOT/'output/run')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--export-model', action='store_true')
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument('--compare', '--compare-fluid', nargs='*', choices=['ellipsoid', 'mesh_surface'],
                            help='Compare any number of methods; no values uses comparison.cases or both defaults')
    comparison.add_argument('--compare-config', nargs='+', type=Path,
                            help='Compare independent YAML configs (same timestep and duration)')
    comparison.add_argument('--compare-patterns', nargs='+', choices=list(patterns()), metavar='PATTERN',
                            help='Display independent robots with the selected motion patterns')
    parser.add_argument('--fluid-model', choices=['ellipsoid', 'mesh_surface'])
    parser.add_argument('--mesh', type=Path, help='Source OBJ (does not modify the original file)')
    parser.add_argument('--mesh-scale', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--mesh-rotation', nargs=3, type=float, metavar=('RX', 'RY', 'RZ'),
                        help='Rotate imported OBJ about fixed X, Y, Z axes, in degrees')
    parser.add_argument('--mesh-center', action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument('--fit-mesh', action='store_true', help='Fit rotated mesh bounds to balloon.dimensions')
    parser.add_argument('--dimensions', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='Physical dimensions in meters')
    args = parser.parse_args()
    if args.list_patterns:
        for name, pattern in patterns().items():
            print(f'{name:12} {pattern["name"]}')
        return
    if args.compare_patterns and (args.pattern or args.mode):
        parser.error('--compare-patterns cannot be combined with --pattern or --mode')
    if args.headless and args.start_paused:
        parser.error('--start-paused requires the viewer (Space is unavailable with --headless)')
    comparing = args.compare is not None or args.compare_config is not None or args.compare_patterns is not None
    if (args.compare is not None or args.compare_config is not None) and args.fluid_model:
        parser.error('Choose comparison methods with --compare or the individual configs')
    if args.focus_case is not None and not comparing:
        parser.error('--focus-case requires comparison mode')
    if args.side_all and args.focus_case is not None:
        parser.error('--side-all and --focus-case cannot be used together')
    try:
        c = configure(load_config(args.config), args)
        cases = ([(name, configure(case, args)) for name, case in
                  cases_from_config(c, args.compare, args.compare_config, args.compare_patterns)] if comparing else None)
        sim = Comparison(cases) if comparing else Simulation(c)
        if args.focus_case is not None and not 1 <= args.focus_case <= len(sim.simulations):
            parser.error('--focus-case must be between 1 and the number of comparison cases')
    except (ValueError, OSError) as error:
        parser.error(str(error))
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'config.yaml').write_text(yaml.safe_dump(c, sort_keys=False))
    if args.export_model:
        if comparing:
            for index, (_, case) in enumerate(cases, 1):
                print(f'Model {index}: {build_model(case, ROOT / "models/comparison" / f"{index:02d}")}')
        else:
            print(f'Model: {build_model(c)}')
    path = args.output/'trajectory.csv'
    logger = ComparisonLogger(sim, args.output) if comparing else Logger(path)
    try:
        logger.record(sim)
        if args.headless:
            if args.start_delay:
                time.sleep(args.start_delay)
            start = time.monotonic()
            while sim.data.time < sim.config['simulation']['duration'] - 1e-10:
                sim.step()
                logger.record(sim)
                if args.realtime:
                    time.sleep(max(0, start + sim.data.time - time.monotonic()))
        else:
            view(sim, logger, args.realtime, args.camera,
                 start_paused=args.start_paused, start_delay=args.start_delay,
                 x_direction=args.x_direction, projection=args.projection,
                 focus_case=args.focus_case, side_all=args.side_all, maximize=not args.windowed)
    finally:
        logger.close()
    if args.plot:
        if comparing:
            for case_path in logger.paths:
                plot(case_path, case_path.parent)
            plot_comparison(logger.paths, sim.names, args.output)
        else:
            plot(path, args.output)
    if comparing:
        for name, case, case_path in zip(sim.names, sim.simulations, logger.paths):
            print(f'{name}: {case.config["physics"].get("fluid_model", "ellipsoid")}; CSV: {case_path}')
        print(f'Completed {sim.data.time:.3f} s for {len(sim.simulations)} cases')
    else:
        print(f'Mass: {sim.model.body_mass.sum():.6f} kg; buoyancy: {sim.forces.sum():.6f} N')
        print(f'Horizontal distance: {sim.horizontal_distance:.3f} m; X displacement: {sim.displacement[0]:+.3f} m')
        print(f'Completed {sim.data.time:.3f} s; CSV: {path}')


if __name__ == '__main__':
    main()
