from copy import deepcopy
import numpy as np
import pytest
from src.model import load_config
from src.simulation import Simulation
from src.controller import targets


def config():
    c = load_config()
    c['physics']['fluid'] = False
    c['control']['mode'] = 'fixed'
    return c


def test_structure_mass_and_force():
    sim = Simulation(config())
    assert (sim.model.nq, sim.model.nv, sim.model.nu) == (11, 10, 4)
    assert sim.model.body_mass.sum() == pytest.approx(.2761)
    assert sim.forces.sum() == pytest.approx(1.205 * .350 * 9.81)
    assert sim.data.qacc[2] > 0


def test_free_fall():
    c = config()
    c['physics']['buoyancy'] = False
    sim = Simulation(c)
    np.testing.assert_allclose(sim.data.qacc[2], -9.81, atol=1e-9)
    z = sim.center_of_mass[2]
    for _ in range(100):
        sim.step()
    assert sim.center_of_mass[2] < z


def test_neutral_center_of_mass_acceleration():
    c = config()
    c['physics']['neutral'] = True
    sim = Simulation(c)
    assert sim.model.body_mass.sum() == pytest.approx(.42175)
    start = sim.center_of_mass.copy()
    for _ in range(100):
        sim.step()
    assert abs(sim.center_of_mass[2] - start[2]) < 2e-6


def test_offset_buoyancy_moment():
    c = config()
    a = Simulation(c)
    c['balloon']['buoyancy_center'] = [0.1, 0, 0]
    b = Simulation(c)
    delta = b.data.qfrc_applied - a.data.qfrc_applied
    assert delta[4] == pytest.approx(-0.1 * b.forces.sum())


def test_internal_motion_conserves_com():
    c = config()
    c['simulation']['gravity'] = 0
    c['simulation']['timestep'] = .0005
    c['control']['mode'] = 'traveling'
    sim = Simulation(c)
    start = sim.center_of_mass.copy()
    for _ in range(4000):
        sim.step()
    # Euler integration has first-order momentum drift; verify bounded error
    # and convergence instead of treating finite-step drift as external thrust.
    error = np.linalg.norm(sim.center_of_mass - start)
    # Use a 0.5 ms step for this two-second accuracy benchmark as the compact
    # reverse-phase gait increases Euler drift. Require convergence at 0.25 ms.
    assert error < .003
    c['simulation']['timestep'] /= 2
    fine = Simulation(c)
    fine_start = fine.center_of_mass.copy()
    for _ in range(8000):
        fine.step()
    assert np.linalg.norm(fine.center_of_mass - fine_start) < .55 * error
    assert np.linalg.norm(sim.data.qpos[7:]) > .1
    assert np.max(np.abs(sim.data.actuator_force)) <= c['joint']['torque_limit'] + 1e-12


def test_fluid_damping_and_direction():
    c = config()
    c['simulation']['gravity'] = 0
    def force(axis, enabled):
        local = deepcopy(c)
        local['physics']['fluid'] = enabled
        sim = Simulation(local)
        sim.data.qvel[axis] = 1
        sim.refresh()
        return (sim.data.qfrc_passive + sim.data.qfrc_applied)[axis]
    assert force(0, False) == pytest.approx(0)
    assert force(0, True) < 0
    assert abs(force(2, True)) > abs(force(0, True))


@pytest.mark.parametrize('mode', ['fixed', 'sine', 'traveling', 'asymmetric'])
def test_control_bounds(mode):
    c = config()['control']
    c['mode'] = mode
    c['amplitude_deg'] = 180
    c['fixed_deg'] = [-200, -30, 40, 200]
    for t in np.linspace(0, 4, 80):
        angles = targets(t, c)
        assert angles.shape == (4,)
        assert np.max(np.abs(angles)) <= np.pi / 2


def test_independent_joint_tracking():
    c = config()
    c['simulation']['gravity'] = 0
    c['control']['fixed_deg'] = [-20, 10, 25, -15]
    sim = Simulation(c)
    for _ in range(2500):
        sim.step()
    np.testing.assert_allclose(sim.data.qpos[7:], np.deg2rad(c['control']['fixed_deg']), atol=.04)


def test_ballast_below_balloon_changes_com_and_inertia():
    c = config()
    c['physics']['neutral'] = True
    lower = Simulation(c)
    c['physics']['ballast_position'] = [0, 0, 0]
    centered = Simulation(c)
    assert lower.center_of_mass[2] < centered.center_of_mass[2]
    assert lower.model.body_mass.sum() == pytest.approx(centered.model.body_mass.sum())
    assert not np.allclose(lower.model.body_inertia, centered.model.body_inertia)


@pytest.mark.parametrize('method', ['ellipsoid', 'mesh_surface'])
def test_default_gait_propels_positive_x_and_phase_reversal_changes_direction(method):
    displacement = []
    for phase in [-45, 45]:
        c = load_config()
        assert c['control']['phase_step_deg'] == -45
        c['physics']['fluid_model'] = method
        c['control']['phase_step_deg'] = phase
        sim = Simulation(c)
        initial = sim.center_of_mass[0]
        for _ in range(round(2 / sim.model.opt.timestep)):
            sim.step()
        displacement.append(sim.center_of_mass[0] - initial)
    assert displacement[0] > .03
    assert displacement[1] < -.03


@pytest.mark.parametrize('angle', [-30., 0., 20.])
def test_straight_trim_equilibrium(angle):
    c = config()
    c['physics']['tilt_deg'] = angle
    sim = Simulation(c)
    assert sim.model.body_mass.sum() == pytest.approx(sim.forces.sum() / 9.81)
    def check_tilt():
        span = sim.data.xpos[sim.body_ids[-1]] - sim.data.xpos[sim.body_ids[0]]
        assert np.rad2deg(np.arctan2(span[2], span[0])) == pytest.approx(angle)

    check_tilt()
    sim.data.qpos[3:7] = [0, 1, 0, 0]
    sim.reset()
    check_tilt()
    applied = sim.data.qfrc_applied - sim.data.qfrc_bias
    np.testing.assert_allclose(applied[:6], 0, atol=1e-7)
    assert sim.model.neq == 4


def test_unreachable_trim_angle():
    c = config()
    c['physics']['tilt_deg'] = 85
    with pytest.raises(ValueError, match='指定角度'):
        Simulation(c)
