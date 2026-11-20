import mujoco
import numpy as np
import pytest

from src.comparison import Comparison, cases_from_config
from src.model import load_config
from src.simulation import Simulation
from src.visualization import initialize_camera, update_camera


@pytest.mark.parametrize('mode,elevation,orthographic', [
    ('overview', -25, 0), ('side', 0, 0), ('follow', -25, 0), ('top', -90, 1),
])
def test_camera_presets_and_projection_switching(mode, elevation, orthographic):
    sim = Comparison(cases_from_config(load_config()))
    camera = mujoco.MjvCamera()
    initialize_camera(sim, camera, 'top')
    initialize_camera(sim, camera, mode)
    assert camera.elevation == elevation
    assert camera.orthographic == orthographic
    if not orthographic:
        assert sim.model.vis.global_.fovy == 45


@pytest.mark.parametrize('direction,sign', [('left', -1), ('right', 1)])
@pytest.mark.parametrize('mode', ['overview', 'side', 'top'])
def test_world_x_projects_to_the_selected_screen_direction(direction, sign, mode):
    sim = Simulation(load_config())
    initial = sim.data.qpos.copy()
    camera = mujoco.MjvCamera()
    initialize_camera(sim, camera, mode, x_direction=direction)
    scene = mujoco.MjvScene(sim.model, maxgeom=10)
    mujoco.mjv_updateCamera(sim.model, sim.data, camera, scene)
    right = np.cross(scene.camera[0].forward, scene.camera[0].up)
    assert sign * right[0] > .99
    np.testing.assert_array_equal(sim.data.qpos, initial)


@pytest.mark.parametrize('mode', ['overview', 'side'])
def test_manual_camera_is_not_overwritten_each_frame(mode):
    sim = Comparison(cases_from_config(load_config()))
    camera = mujoco.MjvCamera()
    zoom = initialize_camera(sim, camera, mode)
    camera.lookat[:] = [4, 5, 6]
    camera.distance = 8
    camera.azimuth = 22
    camera.elevation = -17
    sim.step()
    update_camera(sim, camera, mode, zoom)
    np.testing.assert_array_equal(camera.lookat, [4, 5, 6])
    assert (camera.distance, camera.azimuth, camera.elevation) == (8, 22, -17)


def test_orthographic_zoom_and_follow_keep_user_orientation():
    sim = Comparison(cases_from_config(load_config()))
    camera = mujoco.MjvCamera()
    zoom = initialize_camera(sim, camera, 'top')
    original_height = sim.model.vis.global_.fovy
    camera.distance *= 2
    update_camera(sim, camera, 'top', zoom)
    assert sim.model.vis.global_.fovy == pytest.approx(2 * original_height)
    initialize_camera(sim, camera, 'side', projection='orthographic', focus=1)
    assert camera.elevation == 0
    assert camera.orthographic
    zoom = initialize_camera(sim, camera, 'follow', focus=1)
    camera.azimuth = 40
    camera.distance = 10
    sim.simulations[1].data.qpos[:3] += [2, 0, 3]
    sim.simulations[1].refresh()
    update_camera(sim, camera, 'follow', zoom, focus=1)
    np.testing.assert_allclose(camera.lookat, sim.centers[1])
    assert camera.azimuth == 40 and camera.distance == 10
