from contextlib import nullcontext

import mujoco
import mujoco.viewer
import numpy as np
import pytest

from src.comparison import Comparison, cases_from_config
from src.main import main
from src.model import load_config
from src.simulation import Simulation
from src.visualization import view, PlaybackClock


class Clock:
    now = 0.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Viewer:
    """Drive the real view/physics loop without a graphics connection."""
    def __init__(self, sim, keys, frames):
        self.sim, self.keys, self.frames = sim, keys, frames
        self.snapshots = []
        self.camera_frames = []
        self.texts = None
        self.text_frames = []
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        self.user_scn = mujoco.MjvScene(sim.model, maxgeom=100)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def is_running(self):
        return len(self.snapshots) < self.frames

    def lock(self):
        return nullcontext()

    def set_texts(self, texts):
        self.texts = texts

    def sync(self):
        self.text_frames.append(self.texts)
        self.snapshots.append((self.sim.data.time, self.sim.data.qpos.copy()))
        self.camera_frames.append((self.cam.elevation, self.cam.orthographic,
                                   self.sim.model.geom_rgba[:, 3].copy()))
        key = self.keys.get(len(self.snapshots))
        if key is not None:
            self.key(key)


def run_view(monkeypatch, sim, keys=None, frames=6, return_viewer=False,
             frame_interval=.002, sync_cost=0., step_cost=0., **options):
    viewer = Viewer(sim, keys or {}, frames)
    clock = Clock()
    monkeypatch.setattr('src.visualization.time.monotonic', clock.monotonic)
    monkeypatch.setattr('src.visualization.time.sleep', clock.sleep)
    monkeypatch.setattr('src.visualization.FRAME_INTERVAL', frame_interval)
    original_sync, original_step = viewer.sync, sim.step

    def sync():
        original_sync()
        clock.now += sync_cost

    def step():
        original_step()
        clock.now += step_cost

    monkeypatch.setattr(viewer, 'sync', sync)
    monkeypatch.setattr(sim, 'step', step)

    def launch(model, data, key_callback):
        viewer.key = key_callback
        return viewer

    monkeypatch.setattr(mujoco.viewer, 'launch_passive', launch)
    monkeypatch.setattr('src.visualization.startup_window', lambda maximized: nullcontext(lambda: None))
    logger = type('Logger', (), {'record': lambda self, sim: None})()
    view(sim, logger, **options)
    return viewer if return_viewer else viewer.snapshots


def test_side_view_cycles_visible_cases_and_returns_to_overview(monkeypatch):
    sim = Comparison(cases_from_config(load_config()))
    viewer = run_view(monkeypatch, sim, keys={2: 258, 4: 48}, camera='side',
                      start_paused=True, return_viewer=True)
    for index, (elevation, orthographic, alpha) in enumerate(viewer.camera_frames):
        if index < 4:
            active = 0 if index < 2 else 1
            assert elevation == 0 and orthographic == 0
            assert np.any(alpha[sim.case_geom_ids[active]] > 0)
            assert np.all(alpha[sim.case_geom_ids[1 - active]] == 0)
        else:
            assert elevation == -25 and orthographic == 0
            assert all(np.any(alpha[ids] > 0) for ids in sim.case_geom_ids)
    assert all(time == 0 for time, _ in viewer.snapshots)


def test_side_all_displays_every_case_and_a_toggles_single_case(monkeypatch):
    sim = Comparison(cases_from_config(load_config()))
    viewer = run_view(monkeypatch, sim, keys={2: ord('A'), 4: ord('A')}, camera='side',
                      start_paused=True, side_all=True, return_viewer=True)
    for index, (elevation, orthographic, alpha) in enumerate(viewer.camera_frames):
        assert elevation == 0
        if index < 2 or index >= 4:
            assert orthographic == 1
            assert all(np.any(alpha[ids] > 0) for ids in sim.case_geom_ids)
        else:
            assert orthographic == 0
            assert np.all(alpha[sim.case_geom_ids[1]] == 0)


@pytest.mark.parametrize('comparing', [False, True])
def test_start_paused_freezes_every_case_until_space(monkeypatch, comparing):
    c = load_config()
    sim = Comparison(cases_from_config(c)) if comparing else Simulation(c)
    initial = sim.data.qpos.copy()
    frames = run_view(monkeypatch, sim, keys={3: 32}, start_paused=True)
    for time, position in frames[:3]:
        assert time == 0
        np.testing.assert_array_equal(position, initial)
    assert frames[3][0] == pytest.approx(sim.model.opt.timestep)
    assert frames[-1][0] > 0
    if comparing:
        assert all(case.data.time == sim.data.time for case in sim.simulations)


def test_start_delay_holds_state_without_consuming_simulation_time(monkeypatch):
    sim = Simulation(load_config())
    initial = sim.data.qpos.copy()
    frames = run_view(monkeypatch, sim, start_delay=.05)
    for time, position in frames[:3]:
        assert time == 0
        np.testing.assert_array_equal(position, initial)
    assert frames[3][0] == pytest.approx(.002)


def test_space_skips_countdown(monkeypatch):
    sim = Simulation(load_config())
    frames = run_view(monkeypatch, sim, keys={1: 32}, start_delay=10)
    assert frames[0][0] == 0
    assert frames[1][0] == pytest.approx(.002)


def test_reset_restores_start_paused_mode(monkeypatch):
    sim = Simulation(load_config())
    frames = run_view(monkeypatch, sim, keys={2: 32, 4: ord('R'), 7: 32},
                      frames=9, start_paused=True)
    assert frames[3][0] > 0
    assert [time for time, _ in frames[4:7]] == [0, 0, 0]
    assert frames[7][0] == pytest.approx(.002)


@pytest.mark.parametrize('options', [
    ['--headless', '--start-paused'], ['--start-delay', '-1'], ['--start-delay', 'nan'],
    ['--start-delay', 'inf'], ['--start-paused', '--start-delay', '1'],
])
def test_invalid_start_options_fail_before_starting(monkeypatch, options):
    monkeypatch.setattr('sys.argv', ['main', *options])
    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 2


def test_single_distance_overlay_is_visible_paused_and_clears_on_reset(monkeypatch):
    sim = Simulation(load_config())
    sim.data.qpos[:3] += [-3, 4, 20]
    sim.refresh()
    viewer = run_view(monkeypatch, sim, keys={2: ord('R')}, frames=4,
                      start_paused=True, return_viewer=True)
    for text in viewer.text_frames[:2]:
        assert text[1] == mujoco.mjtGridPos.mjGRID_BOTTOMLEFT
        fields = dict(zip(text[2].splitlines(), text[3].splitlines()))
        assert fields['Horizontal distance'] == '5.000 m'
        assert fields['X displacement'] == '-3.000 m'
    for text in viewer.text_frames[2:]:
        fields = dict(zip(text[2].splitlines(), text[3].splitlines()))
        assert fields['Horizontal distance'] == '0.000 m'
        assert fields['X displacement'] == '+0.000 m'


def test_playback_clock_excludes_paused_time_and_resets():
    clock = PlaybackClock(10)
    clock.update(20, True)
    assert clock.elapsed == 0
    clock.update(22, False)
    assert clock.elapsed == 2
    clock.update(100, True)
    assert clock.elapsed == 2
    clock.update(101, True)
    assert clock.elapsed == 3
    assert PlaybackClock(101).elapsed == 0


@pytest.mark.parametrize('comparing', [False, True])
def test_time_overlay_is_visible_in_single_and_comparison_and_resets(monkeypatch, comparing):
    c = load_config()
    sim = Comparison(cases_from_config(c)) if comparing else Simulation(c)
    viewer = run_view(monkeypatch, sim, keys={2: 32, 4: ord('R')}, frames=6,
                      start_paused=True, return_viewer=True)
    for index, text in enumerate(viewer.text_frames):
        fields = dict(zip(text[2].splitlines(), text[3].splitlines()))
        assert fields['Simulation time'] == f'{viewer.snapshots[index][0]:.2f} s'
        if index < 2 or index >= 4:
            assert fields['Simulation time'] == '0.00 s'
            assert fields['Running wall time'] == '0.00 s'
            assert fields['Target cycle'] == '0.00 / 2 s'


def test_realtime_batches_physics_to_avoid_one_render_per_timestep(monkeypatch):
    c = load_config()
    c['physics']['fluid'] = False
    c['simulation']['duration'] = 2.
    sim = Simulation(c)
    viewer = run_view(monkeypatch, sim, frames=130, frame_interval=1/60,
                      sync_cost=.008, return_viewer=True)
    # Even if each render takes 8 ms, 2 simulated seconds fit in 120 display frames.
    assert viewer.snapshots[119][0] == pytest.approx(2)
    assert viewer.snapshots[-1][0] == pytest.approx(2)
    assert viewer.snapshots[0][0] == pytest.approx(.016)
    fields = dict(zip(viewer.text_frames[119][2].splitlines(), viewer.text_frames[119][3].splitlines()))
    assert fields['Target cycle'] == '0.00 / 2 s'


def test_slow_physics_is_reported_in_simulation_time_wall_time_and_speed(monkeypatch):
    c = load_config()
    c['physics']['fluid'] = False
    sim = Simulation(c)
    viewer = run_view(monkeypatch, sim, frames=30, frame_interval=1/60,
                      step_cost=.004, sync_cost=.001, return_viewer=True)
    text = viewer.text_frames[-1]
    fields = dict(zip(text[2].splitlines(), text[3].splitlines()))
    assert float(fields['Running wall time'].split()[0]) > float(fields['Simulation time'].split()[0])
    assert float(fields['Playback speed'].removesuffix('x')) < .6


def test_viewer_finishes_when_duration_is_not_a_multiple_of_timestep(monkeypatch):
    c = load_config()
    c['simulation']['duration'] = .005
    sim = Simulation(c)
    viewer = run_view(monkeypatch, sim, frames=8, return_viewer=True)
    assert viewer.snapshots[-1][0] == pytest.approx(.006)
    assert all(time == pytest.approx(.006) for time, _ in viewer.snapshots[2:])


@pytest.mark.parametrize('comparing', [False, True])
def test_partial_replay_resumes_without_recomputing(monkeypatch, comparing):
    def simulation():
        config = load_config()
        return Comparison(cases_from_config(config)) if comparing else Simulation(config)

    sim = simulation()
    reference = simulation()
    step = sim.step
    computed = []

    def record_step():
        computed.append(sim.data.time)
        step()

    monkeypatch.setattr(sim, 'step', record_step)
    frames = run_view(monkeypatch, sim, keys={6: ord('R'), 9: ord('R')}, frames=28)
    assert frames[6][0] < frames[5][0]
    assert frames[9][0] < frames[8][0]
    assert frames[-1][0] > frames[5][0]
    assert all(a < b for a, b in zip(computed, computed[1:]))
    for _ in computed:
        reference.step()
    np.testing.assert_array_equal(reference.data.qpos, sim.data.qpos)
