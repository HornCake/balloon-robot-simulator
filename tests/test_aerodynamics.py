import mujoco
import numpy as np
import pytest

from src.model import load_config
from src.simulation import Simulation


def surface_config():
    c = load_config()
    c['simulation']['gravity'] = 0
    c['physics'].update(fluid_model='mesh_surface', viscosity=0)
    c['control']['mode'] = 'fixed'
    return c


@pytest.mark.parametrize('reverse', [False, True])
def test_cube_pressure_matches_analytic_force(tmp_path, reverse):
    # A cube has exactly one unit-area windward side per translation axis.
    vertices = [(-.5, -.5, -.5), (.5, -.5, -.5), (.5, .5, -.5), (-.5, .5, -.5),
                (-.5, -.5, .5), (.5, -.5, .5), (.5, .5, .5), (-.5, .5, .5)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
             (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    obj = tmp_path / 'cube.obj'
    obj.write_text('\n'.join(['v ' + ' '.join(map(str, v)) for v in vertices]
                            + ['f ' + ' '.join(str(i + 1) for i in (f[::-1] if reverse else f))
                               for f in faces]) + '\n')
    c = surface_config()
    c['balloon']['mesh_file'] = str(obj)
    sim = Simulation(c)
    velocity = np.array([1., -2., 3.])
    sim.data.qvel[:3] = velocity
    sim.refresh()
    expected = -5 * .5 * c['physics']['air_density'] * np.abs(velocity) * velocity
    np.testing.assert_allclose(sim.data.qfrc_applied[:3], expected, atol=1e-6)
    # Native quadratic resistance must not duplicate the custom surface force.
    np.testing.assert_allclose(sim.data.qfrc_passive, 0, atol=1e-10)


def test_surface_drag_dissipates_translational_and_joint_motion():
    sim = Simulation(surface_config())
    sim.data.qpos[7:] = [.3, -.2, .1, -.1]
    sim.data.qvel[:] = [1, -.3, .7, .4, -.8, .2, -.5, .3, -.1, .2]
    sim.refresh()
    assert sim.data.qfrc_applied @ sim.data.qvel < 0
    force = sim.data.qfrc_applied.copy()
    sim.refresh()
    np.testing.assert_allclose(sim.data.qfrc_applied, force)


def test_mesh_rotation_and_scale_affect_drag():
    c = surface_config()
    original = Simulation(c)
    original.data.qvel[0] = 1
    original.refresh()
    force = original.data.qfrc_applied[:3].copy()
    rotated = Simulation(c)
    rotated.data.qpos[3:7] = [np.sqrt(.5), 0, 0, np.sqrt(.5)]
    rotated.data.qvel[1] = 1
    rotated.refresh()
    np.testing.assert_allclose(rotated.data.qfrc_applied[:3], [-force[1], force[0], force[2]], atol=1e-8)
    c['balloon']['mesh_scale'] = [.5, .5, .5]
    smaller = Simulation(c)
    smaller.data.qvel[0] = 1
    smaller.refresh()
    np.testing.assert_allclose(smaller.data.qfrc_applied[:3], .25 * force, atol=1e-8)


def test_wind_uses_relative_velocity_and_drag_scales_quadratically():
    c = surface_config()
    c['physics']['wind'] = [1, 0, 0]
    sim = Simulation(c)
    assert sim.data.qfrc_applied[0] > 0
    force = sim.data.qfrc_applied.copy()
    sim.data.qvel[0] = 1
    sim.refresh()
    np.testing.assert_allclose(sim.data.qfrc_applied, 0, atol=1e-10)
    c['physics']['wind'] = [2, 0, 0]
    sim.data.qvel[0] = 0
    sim.refresh()
    np.testing.assert_allclose(sim.data.qfrc_applied, 4 * force)


@pytest.mark.parametrize('model', ['mesh_surface', 'ellipsoid'])
def test_fluid_switch_and_legacy_model(model):
    c = surface_config()
    c['physics']['fluid_model'] = model
    sim = Simulation(c)
    sim.data.qvel[0] = 1
    sim.refresh()
    assert (sim.data.qfrc_applied + sim.data.qfrc_passive)[0] < 0
    c['physics']['fluid'] = False
    off = Simulation(c)
    off.data.qvel[0] = 1
    off.refresh()
    np.testing.assert_allclose(off.data.qfrc_applied + off.data.qfrc_passive, 0, atol=1e-12)


@pytest.mark.parametrize('seed', range(5))
def test_surface_force_and_torque_match_direct_panel_sum(seed):
    """Compare generalized forces against the original per-face formulation."""
    rng = np.random.default_rng(seed)
    c = surface_config()
    c['physics']['wind'] = rng.normal(size=3).tolist()
    sim = Simulation(c)
    sim.data.qpos[3:7] = rng.normal(size=4)
    sim.data.qpos[3:7] /= np.linalg.norm(sim.data.qpos[3:7])
    sim.data.qpos[7:] = rng.uniform(-1, 1, 4)
    sim.data.qvel[:] = rng.normal(size=sim.model.nv)
    sim.refresh()
    expected = np.zeros(sim.model.nv)
    model, data = sim.model, sim.data
    for geom in sim.surface_drag.geoms:
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start + count].astype(float)
        start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        triangles = vertices[model.mesh_face[start:start + count]]
        centers = triangles.mean(axis=1)
        area_vectors = np.cross(triangles[:, 1] - triangles[:, 0],
                                triangles[:, 2] - triangles[:, 0]) / 2
        area_vectors *= np.sign(np.einsum('ij,ij->', triangles[:, 0], area_vectors))
        areas = np.linalg.norm(area_vectors, axis=1)
        valid = areas > 1e-14
        centers, normals, areas = centers[valid], area_vectors[valid] / areas[valid, None], areas[valid]
        rotation = data.geom_xmat[geom].reshape(3, 3)
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_GEOM, geom, velocity, 0)
        local_velocity = (rotation.T @ (velocity[3:] - c['physics']['wind'])
                          + np.cross(rotation.T @ velocity[:3], centers))
        speed = np.maximum(np.einsum('ij,ij->i', local_velocity, normals), 0)
        forces = (-.5 * c['physics']['air_density'] * areas * speed**2)[:, None] * normals
        mujoco.mj_applyFT(model, data, rotation @ forces.sum(axis=0),
                         rotation @ np.cross(centers, forces).sum(axis=0),
                         data.geom_xpos[geom], int(model.geom_bodyid[geom]), expected)
    np.testing.assert_allclose(data.qfrc_applied, expected, rtol=1e-12, atol=1e-12)
