import numpy as np
import pytest

from src.controller import targets
from src.search_motion import candidates, control_for, evaluate, search_step
from src.model import load_config
from src.simulation import Simulation


def test_search_has_unique_cases_and_covers_requested_ranges():
    cases = candidates()
    assert len({s['id'] for s in cases}) == len(cases) == 2415
    assert {s['amplitude'] for s in cases} == {10, 20, 30, 45, 60, 75, 90}
    assert {s['period'] for s in cases} == {.5, 1, 2, 4, 8}


@pytest.mark.parametrize('family', ['sine', 'asym25', 'asym75', 'triangle', 'harmonic', 'tail', 'head', 'air-06'])
def test_search_waveforms_are_periodic_and_bounded(family):
    c = control_for(dict(family=family, amplitude=60, period=2., phase=-60))
    for t in np.linspace(.01, 2., 101):
        np.testing.assert_allclose(targets(t, c), targets(t+2., c), atol=1e-12)
        assert np.max(np.abs(targets(t, c))) <= np.deg2rad(60.1)


def test_evaluation_reports_com_speed_over_actual_duration():
    result = evaluate((dict(id='test', family='sine', amplitude=20, period=.02, phase=-60),
                       'ellipsoid', .04, .002, True))
    assert 'error' not in result
    assert result['speed'] == pytest.approx(result['dx']/.04)
    assert np.isfinite(result['late_speed'])


@pytest.mark.parametrize('fluid', ['mesh_surface', 'ellipsoid'])
def test_search_step_preserves_simulation_trajectory(fluid):
    cfg = load_config()
    cfg['physics'].update(fluid_model=fluid, neutral=True)
    cfg['control'].update(amplitude_deg=90, period=.5)
    original, fast = Simulation(cfg), Simulation(cfg)
    for _ in range(1000):
        original.step()
        search_step(fast)
    np.testing.assert_array_equal(fast.data.qpos, original.data.qpos)
    np.testing.assert_array_equal(fast.data.qvel, original.data.qvel)
