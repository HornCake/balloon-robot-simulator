"""Run with uv run python -m src.main (macOS viewer: uv run mjpython -m src.main)."""
import argparse
import math
import sys
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
    if args.ballast_ratios is not None:
        c['physics'].pop('tilt_deg', None)
        c['physics']['ballast_ratios'] = args.ballast_ratios
    if args.tilt_deg is not None:
        c['physics']['tilt_deg'] = args.tilt_deg
        c['physics']['neutral'] = True
        if not (args.mode or args.pattern or args.compare_patterns is not None or args.compare_config):
            clear_pattern(c['control'])
            c['control'].update(mode='fixed', fixed_deg=[0.] * 4)
    if args.neutral or args.ballast_ratios is not None:
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
    parser = argparse.ArgumentParser(description='5連結ヘリウムバルーンのシミュレーション')
    parser.add_argument('-i', '--interactive', action='store_true', help='メニューで起動時の設定を選ぶ。')
    parser.add_argument('--config', type=Path, default=ROOT/'config/default.yaml', help='シミュレーションの設定を指定したYAMLファイルから読み込む。')
    parser.add_argument('--headless', action='store_true', help='ビューアーを開かずにシミュレーションを実行する。')
    parser.add_argument('--realtime', action='store_true', help='シミュレーション内の時間が実時間と同じ速さで進むように調整する。')
    parser.add_argument('--windowed', action='store_true', help='ビューアーを従来のウィンドウサイズで開く。')
    startup = parser.add_mutually_exclusive_group()
    startup.add_argument('--start-paused', action='store_true', help='一時停止した状態でビューアーを開く。スペースキーで計算を開始する。')
    startup.add_argument('--start-delay', type=nonnegative_seconds, default=0, metavar='SECONDS',
                         help='指定した秒数だけ初期状態で待ってから計算を開始する。')
    parser.add_argument('--duration', type=float, help='シミュレーション内の計算時間を秒で指定する。')
    parser.add_argument('--no-buoyancy', action='store_true', help='バルーンにかかる浮力を無効にする。')
    parser.add_argument('--no-fluid', action='store_true', help='空気から受ける力を無効にする。')
    parser.add_argument('--neutral', action='store_true', help='重りを加え、浮力と重力がつり合うようにする。')
    trim = parser.add_mutually_exclusive_group()
    trim.add_argument('--tilt-deg', type=float, metavar='度',
                      help='指定角度で開始し、その角度でつり合うように重りを自動配分する。正なら5番側が上がる。単独指定で関節を直線に固定。')
    trim.add_argument('--ballast-ratios', nargs=5, type=float, metavar='比',
                        help='中性浮力の重りを1〜5番（+X方向）の順に配分する。例: 3 2 1 1 1 で1番側が下がる。')
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument('--mode', choices=['fixed','sine','traveling','asymmetric'],
                        help='関節の制御方式を固定・正弦波・進行波・非対称波から選ぶ。')
    motion.add_argument('--pattern', choices=list(patterns()), help='登録済みの関節動作パターンを1つ選ぶ。')
    parser.add_argument('--list-patterns', action='store_true', help='登録済みの動作パターンを一覧表示して終了する。')
    parser.add_argument('--motion-scale', type=float,
                        help='全関節の目標角度に0以上の倍率を掛け、その後±90度に収める。')
    parser.add_argument('--period', type=float, metavar='SECONDS',
                        help='動作1周期の長さを秒で指定する。既定の設定ファイルでは2秒。')
    parser.add_argument('--phase-step-deg', type=float,
                        help='隣り合う関節の位相差を度で指定する。既定の進行波は−45度で+X方向に進む。')
    parser.add_argument('--camera', choices=['overview','side','follow','top'], default='overview',
                        help='起動時のカメラを斜め・真横・追従・真上から選ぶ。')
    parser.add_argument('--projection', choices=['perspective','orthographic'],
                        help='透視投影か平行投影を選ぶ。既定では真上が平行投影、それ以外が透視投影。')
    parser.add_argument('--x-direction', choices=['left','right'], default='right',
                        help='ビューアーを開いたとき、ワールド座標の+X方向を画面の左と右のどちらに向けるか指定する。')
    parser.add_argument('--focus-case', type=int, metavar='N', help='1から数えて指定した比較ケースだけを表示する。')
    parser.add_argument('--side-all', action='store_true', help='全比較ケースを真横から同時に表示する。ビューアーではAキーで切り替える。')
    parser.add_argument('--output', type=Path, default=ROOT/'output/run', help='軌跡データ・設定・グラフの保存先を指定する。')
    parser.add_argument('--plot', action='store_true', help='実行後にシミュレーション結果のグラフを生成する。')
    parser.add_argument('--export-model', action='store_true', help='生成したMuJoCoのXMLとメッシュをmodels/に保存する。')
    comparison = parser.add_mutually_exclusive_group()
    comparison.add_argument('--compare-fluid', '--compare', dest='compare', nargs='*', choices=['ellipsoid', 'mesh_surface'],
                            help='空力モデルを比較する。値を省くと設定ファイルの比較ケースを使い、未設定なら両方式を使う。')
    comparison.add_argument('--compare-config', nargs='+', type=Path,
                            help='複数のYAML設定を比較する。時間刻みと計算時間はすべて同じ値にする。')
    comparison.add_argument('--compare-patterns', nargs='+', choices=list(patterns()), metavar='PATTERN',
                            help='指定した動作パターンごとにロボットを個別に計算し、並べて表示する。')
    parser.add_argument('--fluid-model', choices=['ellipsoid', 'mesh_surface'], help='空気抵抗の計算方式を楕円体とメッシュ表面から選ぶ。')
    parser.add_argument('--mesh', type=Path, help='バルーンの形状に使うOBJファイルを指定する。元ファイルは保持される。')
    parser.add_argument('--mesh-scale', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='読み込んだOBJの拡大・縮小倍率をX、Y、Z軸の順に指定する。')
    parser.add_argument('--mesh-rotation', nargs=3, type=float, metavar=('RX', 'RY', 'RZ'),
                        help='固定したX、Y、Z軸まわりのOBJの回転角を度で指定する。')
    parser.add_argument('--mesh-center', action=argparse.BooleanOptionalAction, default=None,
                        help='メッシュを囲む直方体の中心がローカル座標の原点に来るように移動する。')
    parser.add_argument('--fit-mesh', action='store_true', help='回転後のメッシュの外寸をバルーンの物理寸法に合わせる。')
    parser.add_argument('--dimensions', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='バルーンの物理寸法をX、Y、Z軸の順にメートルで指定する。')
    args = parser.parse_args()
    if args.list_patterns:
        for name, pattern in patterns().items():
            print(f'{name:12} {pattern["name"]}')
        return
    if args.interactive:
        from .startup_menu import select_startup
        try:
            select_startup(args, parser, patterns(), sys.stdin.isatty() and sys.stdout.isatty())
        except (KeyboardInterrupt, EOFError):
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
