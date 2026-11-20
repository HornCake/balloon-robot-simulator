import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from src.model import ROOT, build_model, load_config, validate_config
from src.simulation import Simulation


def test_ground_is_a_visual_reference():
    sim = Simulation(load_config())
    ground = sim.model.geom('ground')
    assert ground.type == mujoco.mjtGeom.mjGEOM_PLANE
    np.testing.assert_array_equal(ground.pos, [0, 0, 0])
    assert ground.contype == ground.conaffinity == 0
    assert sim.model.mat_texid[ground.matid[0], mujoco.mjtTextureRole.mjTEXROLE_RGB] >= 0


def test_custom_obj_is_copied_scaled_and_keeps_physics(tmp_path):
    c = load_config()
    default = Simulation(c)
    source = ROOT / 'assets/balloon.obj'
    original = source.read_bytes()
    c['balloon']['mesh_file'] = 'assets/balloon.obj'
    c['balloon']['mesh_scale'] = [0.8, 0.9, 1.1]
    path = build_model(c, tmp_path)
    assert source.read_bytes() == original
    prepared = tmp_path / ET.parse(path).find('asset/mesh').get('file')
    def vertices(text):
        return np.array([[float(x) for x in line.split()[1:4]]
                         for line in text.splitlines() if line.startswith('v ')])
    np.testing.assert_allclose(np.ptp(vertices(prepared.read_text()), axis=0),
                               np.ptp(vertices(original.decode()), axis=0) * [.8, .9, 1.1])
    custom = Simulation(c)
    np.testing.assert_allclose(custom.model.body_mass, default.model.body_mass)
    np.testing.assert_allclose(custom.model.body_inertia, default.model.body_inertia)
    np.testing.assert_allclose(custom.data.qacc, default.data.qacc)


@pytest.mark.parametrize('scale', [[0, 1, 1], [-1, 1, 1], [1, 1], [1, 1, np.nan]])
def test_invalid_mesh_scale_is_rejected(tmp_path, scale):
    c = load_config()
    c['balloon'].update(mesh_file='assets/balloon.obj', mesh_scale=scale)
    with pytest.raises(ValueError, match='mesh_scale'):
        build_model(c, tmp_path)


@pytest.mark.parametrize('section, field, value', [
    ('control', 'amplitude_deg', np.nan),
    ('control', 'amplitude_deg', np.inf),
    ('simulation', 'initial_height', np.nan),
    ('simulation', 'initial_height', np.inf),
    ('physics', 'fluid_coefficients', [-.5, .25, 1.5, 1., 1.]),
])
def test_invalid_physical_inputs_are_rejected(section, field, value):
    config = load_config()
    config[section][field] = value
    with pytest.raises(ValueError, match=field):
        validate_config(config)


def test_config_with_japanese_names_uses_utf8(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text(f'extends: {ROOT / "config/default.yaml"}\n'
                    'comparison:\n  cases:\n    - name: 表面抵抗\n', encoding='utf-8')
    assert load_config(path)['comparison']['cases'][0]['name'] == '表面抵抗'
