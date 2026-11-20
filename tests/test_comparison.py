from copy import deepcopy
import csv
from pathlib import Path

import mujoco
import numpy as np
import pytest
import yaml

from src.comparison import Comparison, ComparisonLogger, cases_from_config
from src.main import main
from src.model import ROOT, load_config
from src.simulation import Simulation
from src.visualization import initialize_camera


def test_arbitrary_cases_are_independent_and_display_offsets_do_not_affect_physics(tmp_path):
    config = load_config()
    config['simulation']['duration'] = .02
    original = deepcopy(config)
    methods = ['ellipsoid', 'mesh_surface', 'mesh_surface', 'ellipsoid', 'mesh_surface']
    comparison = Comparison(cases_from_config(config, methods))
    references = [Simulation(deepcopy(sim.config)) for sim in comparison.simulations]
    assert config == original
    assert comparison.model.nq == sum(sim.model.nq for sim in references)
    assert comparison.model.opt.density == comparison.model.opt.viscosity == 0
    assert len(list(comparison.labels())) == 5
    np.testing.assert_array_equal(comparison.offsets[:, [0, 2]], 0)
    assert np.all(np.diff(comparison.offsets[:, 1]) > 0)
    logger = ComparisonLogger(comparison, tmp_path)
    try:
        logger.record(comparison)
        for _ in range(5):
            comparison.step()
            logger.record(comparison)
            for reference in references:
                reference.step()
        for index, (sim, reference) in enumerate(zip(comparison.simulations, references)):
            np.testing.assert_allclose(sim.data.qpos, reference.data.qpos, atol=1e-12)
            root = comparison.model.body(f'case_{index}_balloon_1').id
            np.testing.assert_allclose(comparison.data.xpos[root], sim.data.xpos[sim.body_ids[0]] + comparison.offsets[index])
        # Mutating the display is isolated from every physical simulation.
        comparison.data.qpos[:3] += 100
        np.testing.assert_allclose(comparison.simulations[0].data.qpos, references[0].data.qpos)
        comparison.reset()
        logger.record(comparison)
        assert all(sim.data.time == 0 for sim in comparison.simulations)
    finally:
        logger.close()
    assert len(set(logger.paths)) == 5
    for path in logger.paths:
        with path.open() as file:
            rows = list(csv.DictReader(file))
        assert float(rows[0]['position_x']) == pytest.approx(0)
        assert float(rows[-1]['time']) == 0
        assert len(rows) == 7


def test_case_overrides_support_different_meshes_without_changing_base():
    c = load_config(ROOT / 'config/comparison.example.yaml')
    cases = cases_from_config(c)
    assert len(cases) == 3
    assert [case['physics']['fluid_model'] for _, case in cases] == ['ellipsoid', 'mesh_surface', 'mesh_surface']
    assert cases[1][1]['balloon']['mesh_file'] != cases[2][1]['balloon']['mesh_file']
    assert not c['balloon']['mesh_fit_dimensions']
    assert cases[2][1]['balloon']['mesh_fit_dimensions']
    assert 'comparison' not in cases[1][1]


def test_new_mesh_cases_default_to_surface_drag_unless_explicitly_overridden():
    c = load_config()
    c['physics']['fluid_model'] = 'ellipsoid'
    c['comparison'] = {'cases': [
        {'name': 'Mesh1', 'balloon': {'mesh_file': 'assets/balloon_87.obj'}},
        {'name': 'Mesh2', 'balloon': {'mesh_file': 'assets/balloon.obj'}},
        {'name': 'Explicit ellipsoid', 'fluid_model': 'ellipsoid',
         'balloon': {'mesh_file': 'assets/balloon_87.obj'}},
    ]}
    cases = cases_from_config(c)
    assert [case['physics']['fluid_model'] for _, case in cases] == ['mesh_surface', 'mesh_surface', 'ellipsoid']


@pytest.mark.parametrize('field,value', [('timestep', .001), ('duration', 1)])
def test_different_clocks_are_rejected(field, value):
    cases = cases_from_config(load_config())
    cases[1][1]['simulation'][field] = value
    with pytest.raises(ValueError, match='same timestep and duration'):
        Comparison(cases)


def test_single_case_and_scene_labels():
    comparison = Comparison(cases_from_config(load_config(), ['mesh_surface']))
    scene = mujoco.MjvScene(comparison.model, maxgeom=10)
    for label, position in comparison.labels():
        geom = scene.geoms[0]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LABEL, np.zeros(3),
                           position, np.eye(3).ravel(), np.ones(4))
        geom.label = label
        assert geom.label == '1: mesh_surface  dx=+0.000m'


def test_y_lanes_fit_different_models_and_camera_preserves_x_reference():
    config = load_config()
    cases = cases_from_config(config, ['mesh_surface'] * 3)
    cases[1][1]['balloon'].update(dimensions=[.45, 1.2, .3], mesh_fit_dimensions=True)
    comparison = Comparison(cases)
    centers = comparison.offsets[:, 1]
    half_widths = np.array([sim.config['balloon']['dimensions'][1] / 2 for sim in comparison.simulations])
    assert np.all(centers[1:] - half_widths[1:] > centers[:-1] + half_widths[:-1])
    initial_target = comparison.camera_target()
    for sim in comparison.simulations:
        sim.data.qpos[:3] += [.4, .2, 10]
        sim.refresh()
    comparison.sync_display()
    np.testing.assert_allclose(comparison.camera_target()[:2], initial_target[:2])
    assert comparison.camera_target()[2] == pytest.approx(initial_target[2] + 10)
    np.testing.assert_allclose(comparison.camera_target(follow=True), comparison.center_of_mass)
    assert all('dx=+0.400m' in label for label, _ in comparison.labels())
    view_height = comparison.view_height(comparison.camera_target())
    comparison.simulations[0].data.qpos[2] += 20
    comparison.simulations[0].refresh()
    comparison.sync_display()
    assert comparison.view_height(comparison.camera_target()) == pytest.approx(view_height)
    scene = mujoco.MjvScene(comparison.model, maxgeom=20)
    comparison.draw_annotations(scene)
    assert scene.ngeom == 6  # three case labels, X=0 line, arrow, +X label
    assert scene.geoms[scene.ngeom - 1].label == '+X'
    comparison.reset()
    assert all('dx=+0.000m' in label for label, _ in comparison.labels())
    camera = mujoco.MjvCamera()
    initialize_camera(comparison, camera, 'top')
    mujoco.mjv_updateCamera(comparison.model, comparison.data, camera, scene)
    height = scene.camera[0].frustum_top - scene.camera[0].frustum_bottom
    assert height == pytest.approx(comparison.view_height(comparison.camera_target()))
    assert height < 10


def test_cli_with_independent_configs_and_global_duration(tmp_path, monkeypatch):
    files = []
    for name, method in [('first', 'ellipsoid'), ('second', 'mesh_surface'), ('third', 'mesh_surface')]:
        path = tmp_path / f'{name}.yaml'
        path.write_text(yaml.safe_dump({'extends': str(ROOT / 'config/default.yaml'),
                                        'physics': {'fluid_model': method}}))
        files.append(str(path))
    output = tmp_path / 'output'
    monkeypatch.setattr('sys.argv', ['main', '--headless', '--compare-config', *files,
                                   '--duration', '.004', '--output', str(output)])
    main()
    manifest = yaml.safe_load((output / 'comparison.yaml').read_text())
    assert len(manifest['cases']) == 3
    for case in manifest['cases']:
        config = yaml.safe_load((output / case['config']).read_text())
        assert config['simulation']['duration'] == .004
        data = np.genfromtxt(output / case['csv'], delimiter=',', names=True)
        assert data['time'][-1] == pytest.approx(.004)


def test_circular_config_inheritance_is_rejected(tmp_path):
    path = tmp_path / 'cycle.yaml'
    path.write_text('extends: cycle.yaml\n')
    with pytest.raises(ValueError, match='Circular'):
        load_config(path)
