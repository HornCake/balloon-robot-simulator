import numpy as np
import pytest

from src.model import load_config
from src.simulation import Simulation
from src.main import main


def test_distance_uses_whole_body_com_excludes_altitude_and_resets():
    sim = Simulation(load_config())
    np.testing.assert_array_equal(sim.displacement, 0)
    sim.data.qpos[:3] += [-3, 4, 10]
    sim.refresh()
    np.testing.assert_allclose(sim.displacement, [-3, 4, 10])
    assert sim.horizontal_distance == pytest.approx(5)
    sim.reset()
    np.testing.assert_array_equal(sim.displacement, 0)
    assert sim.horizontal_distance == 0


def test_single_headless_run_prints_distance(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr('sys.argv', ['main', '--headless', '--duration', '.004', '--output', str(tmp_path)])
    main()
    output = capsys.readouterr().out
    assert 'Horizontal distance:' in output
    assert 'X displacement:' in output
