import time
import mujoco
import numpy as np
from .window import startup_window

FRAME_INTERVAL = 1 / 60


class PlaybackClock:
    """Active wall time, excluding initial holds and pauses; reset with R."""
    def __init__(self, now):
        self.elapsed = 0.
        self.previous = now
        self.running = False

    def update(self, now, running):
        if self.running:
            self.elapsed += max(0., now - self.previous)
        self.previous, self.running = now, running


def motion_period(control):
    if control['mode'] == 'fixed':
        return None
    if control.get('period') is not None:
        return control['period']
    if control['mode'] == 'expressions':
        return control.get('expression_period')
    return 1 / control['frequency'] if control['frequency'] > 0 else None


def camera_target(sim, focus=None):
    if focus is not None and hasattr(sim, 'simulations'):
        return sim.centers[focus].copy()
    return sim.center_of_mass.copy()


def initialize_camera(sim, cam, mode, x_direction='right', projection=None, focus=None):
    """Apply a preset only when requested; leave subsequent mouse motion alone."""
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = camera_target(sim, focus)
    cam.azimuth = -90 if x_direction == 'left' else 90
    cam.elevation = {'overview': -25, 'side': 0, 'follow': -25, 'top': -90}[mode]
    cam.orthographic = (projection == 'orthographic' if projection is not None else mode == 'top')
    if hasattr(sim, '_view_radii'):
        radius = sim._view_radii(cam.lookat, focus=focus)
        cam.distance = max(6., 3 * np.linalg.norm(radius))
        azimuth, elevation = np.deg2rad([cam.azimuth, cam.elevation])
        up = np.abs([np.cos(azimuth) * np.sin(elevation),
                     np.sin(azimuth) * np.sin(elevation), np.cos(elevation)])
        right = np.abs([np.sin(azimuth), np.cos(azimuth), 0])
        height = max(3., 2.2 * max(up @ radius + .6, (right @ radius + .3) * .75))
    else:
        cam.distance = 6
        height = 4.
    sim.model.vis.global_.fovy = height if cam.orthographic else 45
    return height / cam.distance  # Orthographic zoom follows mouse distance.


def update_camera(sim, cam, mode, orthographic_scale, focus=None):
    if mode == 'follow':
        cam.lookat[:] = camera_target(sim, focus)
    elif mode == 'top':
        # Keep the user's X/Y pan; only follow altitude in top view.
        simulations = getattr(sim, 'simulations', [sim])
        if focus is not None:
            simulations = [simulations[focus]]
        cam.lookat[2] = max(np.max(case.data.xpos[case.body_ids, 2]) for case in simulations)
    if cam.orthographic:
        sim.model.vis.global_.fovy = max(.01, orthographic_scale * cam.distance)


def status_overlay(sim, wall_time=0., focus=None):
    labels = ['Simulation time', 'Running wall time', 'Playback speed']
    values = [f'{sim.data.time:.2f} s', f'{wall_time:.2f} s',
              f'{sim.data.time / wall_time:.2f}x' if wall_time > 1e-9 else '--']
    cases = getattr(sim, 'simulations', [sim])
    periods = [motion_period(case.config['control']) for case in cases]
    period = periods[focus] if focus is not None else periods[0] if all(p == periods[0] for p in periods) else None
    if period is not None:
        phase = sim.data.time % period
        if np.isclose(phase, period, rtol=0, atol=1e-9):
            phase = 0.
        labels.append('Target cycle')
        values.append(f'{phase:.2f} / {period:g} s')
    case = cases[focus] if focus is not None else sim if not hasattr(sim, 'simulations') else None
    if case is not None:
        labels.extend(['Horizontal distance', 'X displacement', 'Initial shape error'])
        values.extend([f'{case.horizontal_distance:.3f} m', f'{case.displacement[0]:+.3f} m',
                       f'{case.initial_shape_error_deg:.1f} deg'])
    return (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
            '\n'.join(labels), '\n'.join(values))


def view(sim, logger, realtime=True, camera='overview', start_paused=False, start_delay=0,
         x_direction='right', projection=None, focus_case=None, side_all=False, maximize=True):
    import mujoco.viewer
    state = {'paused': start_paused, 'waiting': start_delay > 0, 'deadline': None,
             'reset': False, 'camera': camera, 'fluid': hasattr(sim, 'simulations'),
             'focus': None if focus_case is None else focus_case - 1, 'side_all': side_all}
    mesh_ids = np.flatnonzero((sim.model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)
                              & (sim.model.geom_group != 3))
    geom_rgba = sim.model.geom_rgba.copy()
    site_rgba = sim.model.site_rgba.copy()

    def active_case():
        if not hasattr(sim, 'simulations'):
            return None
        # A true side view projects Y lanes onto each other; start with case 1.
        if state['camera'] == 'side':
            if state['side_all']:
                return None
            return 0 if state['focus'] is None else state['focus']
        return state['focus']

    def key(keycode):
        if keycode == 32:
            if state['waiting']:
                state['waiting'] = False  # Space can skip the initial countdown.
                state['paused'] = False
            else:
                state['paused'] = not state['paused']
        elif keycode in (ord('R'), ord('r')):
            state['reset'] = True
        elif keycode in (ord('F'), ord('f')):
            state['fluid'] = not state['fluid']
        elif keycode in (49, 50, 51, 52):
            state['camera'] = {49:'overview', 50:'side', 51:'follow', 52:'top'}[keycode]
        elif keycode in (91, 93, 258) and hasattr(sim, 'simulations'):
            current = active_case()
            direction = -1 if keycode == 91 else 1
            start = (-1 if direction > 0 else 0) if current is None else current
            state['focus'] = (start + direction) % len(sim.simulations)
            state['side_all'] = False
        elif keycode in (ord('A'), ord('a')) and hasattr(sim, 'simulations'):
            state['side_all'] = not state['side_all']
        elif keycode == 48 and hasattr(sim, 'simulations'):
            state['focus'] = None
            if state['camera'] == 'side':
                state['camera'] = 'overview'

    with startup_window(maximize) as resize, mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=key) as viewer:
        resize()
        previous_camera = None
        previous_fluid = None
        previous_focus = object()
        previous_overlay = None
        orthographic_scale = 1.
        # Start counting after the window is open, so launch time is not charged.
        state['deadline'] = time.monotonic() + start_delay
        playback_clock = PlaybackClock(time.monotonic())
        if start_paused:
            print('Paused at t=0. Press Space to start.')
        elif start_delay:
            print(f'Holding initial state for {start_delay:g} s. Press Space to start now.')
        print('F: toggle air-resistance geometry (orange; model approximation applies)')
        print('1: perspective overview; 2: true side; 3: follow; 4: top; Tab / [ / ]: case; A: all in side view; 0: all in overview')
        while viewer.is_running():
            start = time.monotonic()
            if state['reset']:
                sim.reset()
                logger.record(sim)  # time returning to zero marks a new episode
                state['reset'] = False
                previous_camera = None
                playback_clock = PlaybackClock(start)
                if start_paused:
                    state['paused'] = True
                elif start_delay:
                    state['paused'] = False
                    state['waiting'] = True
                    state['deadline'] = start + start_delay
            if state['waiting'] and start >= state['deadline']:
                state['waiting'] = False
            finished = sim.data.time >= sim.config['simulation']['duration'] - 1e-10
            running = not state['paused'] and not state['waiting'] and not finished
            playback_clock.update(start, running)
            if running:
                # Batch physics steps between display frames. Updating the
                # viewer every 2 ms made rendering throttle simulation time.
                target_time = (playback_clock.elapsed + FRAME_INTERVAL if realtime
                               else sim.config['simulation']['duration'])
                target_time = min(target_time, sim.config['simulation']['duration'])
                max_steps = max(1, int(np.ceil(FRAME_INTERVAL / sim.model.opt.timestep)))
                for _ in range(max_steps):
                    last_step = target_time >= sim.config['simulation']['duration'] - 1e-10
                    if (sim.data.time + sim.model.opt.timestep > target_time + 1e-10
                            and not last_step):
                        break
                    if sim.data.time >= sim.config['simulation']['duration'] - 1e-10:
                        break
                    sim.step()
                    logger.record(sim)
                    if time.monotonic() - start >= FRAME_INTERVAL:
                        break
            finished = sim.data.time >= sim.config['simulation']['duration'] - 1e-10
            playback_clock.update(time.monotonic(), running and not finished)
            with viewer.lock():
                focus = active_case()
                side_overlay = (hasattr(sim, 'simulations') and state['camera'] == 'side'
                                and state['side_all'])
                if hasattr(sim, 'draw_annotations'):
                    sim.draw_annotations(viewer.user_scn, focus=focus, x_direction=x_direction,
                                         top=state['camera'] == 'top', side_overlay=side_overlay)
                if (state['fluid'] != previous_fluid or focus != previous_focus
                        or side_overlay != previous_overlay):
                    viewer.opt.geomgroup[3] = int(state['fluid'])
                    sim.model.geom_rgba[:] = geom_rgba
                    sim.model.site_rgba[:] = site_rgba
                    if state['fluid']:
                        sim.model.geom_rgba[mesh_ids, 3] = .12
                    if side_overlay:
                        for index, ids in enumerate(sim.case_geom_ids):
                            sim.model.geom_rgba[ids, :3] = sim.case_color(index)
                            sim.model.geom_rgba[ids, 3] = .35
                        if state['fluid']:
                            sim.model.geom_rgba[mesh_ids, 3] = .1
                    if focus is not None:
                        for index in range(len(sim.simulations)):
                            if index != focus:
                                sim.model.geom_rgba[sim.case_geom_ids[index], 3] = 0
                                sim.model.site_rgba[sim.case_site_ids[index], 3] = 0
                    previous_fluid = state['fluid']
                    previous_overlay = side_overlay
                if state['camera'] != previous_camera or focus != previous_focus:
                    selected_projection = ('orthographic' if side_overlay and projection is None else projection)
                    orthographic_scale = initialize_camera(sim, viewer.cam, state['camera'],
                                                           x_direction, selected_projection, focus)
                    if side_overlay and viewer.cam.orthographic:
                        radius = sim._view_radii(viewer.cam.lookat)
                        sim.model.vis.global_.fovy = max(sim.model.vis.global_.fovy,
                            2.2 * (radius[2] + .6 + .23 * len(sim.simulations)))
                        orthographic_scale = sim.model.vis.global_.fovy / viewer.cam.distance
                    previous_camera = state['camera']
                previous_focus = focus
                update_camera(sim, viewer.cam, state['camera'], orthographic_scale, focus)
            viewer.set_texts(status_overlay(sim, playback_clock.elapsed, focus))
            viewer.sync()
            if state['paused'] or state['waiting'] or finished:
                playback_clock.update(time.monotonic(), False)
            delay = FRAME_INTERVAL - (time.monotonic()-start) if realtime else 0
            if state['paused'] or state['waiting'] or finished:
                delay = 0.02
            if delay > 0:
                time.sleep(delay)
    sim.model.geom_rgba[:] = geom_rgba
    sim.model.site_rgba[:] = site_rgba


def plot(csv_path, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d = np.atleast_1d(np.genfromtxt(csv_path, delimiter=',', names=True))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for a in 'xyz':
        axes[0,0].plot(d['time'], d[f'position_{a}'], label=a.upper())
    axes[0,0].set(xlabel='Time [s]', ylabel='Root position [m]')
    for i in range(1,5):
        line, = axes[0,1].plot(d['time'], np.rad2deg(d[f'joint_{i}_angle']), label=f'Joint {i}')
        axes[0,1].plot(d['time'], np.rad2deg(d[f'joint_{i}_target']), '--', color=line.get_color(), alpha=0.5)
    axes[0,1].set(xlabel='Time [s]', ylabel='Angle [deg] (dashed: target)')
    axes[1,0].remove()
    trajectory = fig.add_subplot(223, projection='3d')
    trajectory.plot(d['com_x'], d['com_y'], d['com_z'])
    trajectory.set(xlabel='COM X [m]', ylabel='COM Y [m]', zlabel='COM Z [m]')
    speed = np.sqrt(sum(d[f'velocity_{a}']**2 for a in 'xyz'))
    axes[1,1].plot(d['time'], speed, label='Root speed')
    axes[1,1].set(xlabel='Time [s]', ylabel='Speed [m/s]')
    for ax in (axes[0,0], axes[0,1], axes[1,1]):
        ax.legend()
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(directory / 'summary.png', dpi=150)
    plt.close(fig)


def plot_comparison(paths, names, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for index, (path, name) in enumerate(zip(paths, names), 1):
        data = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))
        label = f'{index}: {name}'
        # R resets append episodes to CSV. Measure displacement from each
        # episode's initial COM rather than connecting across the reset.
        starts = np.r_[0, np.flatnonzero(np.diff(data['time']) <= 0) + 1]
        ends = np.r_[starts[1:], len(data)]
        speed = np.sqrt(sum(data[f'velocity_{axis}']**2 for axis in 'xyz'))
        for episode, (start, end) in enumerate(zip(starts, ends)):
            segment = data[start:end]
            legend = label if episode == 0 else '_nolegend_'
            color = f'C{(index - 1) % 10}'
            axes[0, 0].plot(segment['time'], segment['com_x'] - segment['com_x'][0], label=legend, color=color)
            axes[0, 1].plot(segment['time'], segment['velocity_x'], label=legend, color=color)
            axes[1, 0].plot(segment['time'], segment['com_z'], label=legend, color=color)
            axes[1, 1].plot(segment['time'], speed[start:end], label=legend, color=color)
    axes[0, 0].set(xlabel='Time [s]', ylabel='COM X displacement [m]')
    axes[0, 1].set(xlabel='Time [s]', ylabel='Root X velocity [m/s]')
    axes[1, 0].set(xlabel='Time [s]', ylabel='COM height [m]')
    axes[1, 1].set(xlabel='Time [s]', ylabel='Root speed [m/s]')
    axes[0, 0].axhline(0, color='0.5', linewidth=.7)
    axes[0, 1].axhline(0, color='0.5', linewidth=.7)
    for axis in axes.flat:
        axis.legend()
        axis.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(directory / 'comparison.png', dpi=150)
    plt.close(fig)
