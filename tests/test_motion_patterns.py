from copy import deepcopy
import csv

import numpy as np
import pytest
import yaml

from src.comparison import Comparison, cases_from_config
from src.controller import targets
from src.main import main
from src.model import load_config, validate_config
from src.motion_patterns import apply_pattern, compile_expressions, patterns


@pytest.mark.parametrize('name', list(patterns()))
def test_registered_studio_commands_are_finite_and_bounded(name):
    control = load_config()['control']
    apply_pattern(control, name)
    samples = np.array([targets(t, control) for t in np.linspace(0, 32, 641)])
    assert samples.shape == (641, 4)
    assert np.all(np.isfinite(samples))
    limit = 45 if name == 'paddle' else 30 if name == 'wave-01' else 20
    assert np.max(np.abs(np.rad2deg(samples))) <= limit + 1e-10
    assert control['period'] == 2.
    for t in (0, .7, 3.2, 6.8):
        np.testing.assert_allclose(targets(t, control), targets(t + 2, control), atol=1e-12)


def test_commands_follow_actual_studio_formulas_instead_of_stale_descriptions():
    control = load_config()['control']
    control['period'] = None  # Verify the unchanged source formulas at their native timing.
    apply_pattern(control, 'slow-sweep')
    np.testing.assert_allclose(np.rad2deg(targets(3, control)), [20] * 4)
    np.testing.assert_allclose(np.rad2deg(targets(4.05, control)), [-20] * 4)
    apply_pattern(control, 'paddle')
    assert np.all(targets(.1, control) < 0)
    np.testing.assert_allclose(np.rad2deg(targets(1, control)), [-45] * 4)
    np.testing.assert_allclose(targets(5, control), 0, atol=1e-12)
    apply_pattern(control, 'air-04')
    np.testing.assert_allclose(np.rad2deg(targets(2, control)), [0, 0, 10, 10])
    apply_pattern(control, 'air-01')
    np.testing.assert_allclose(np.rad2deg(targets(0, control)),
                               [0, -5 * np.sqrt(3), -7.5 * np.sqrt(3), 0], atol=1e-12)


@pytest.mark.parametrize('expression', [
    'x(n,t) = __import__("os")', 'x(n,t) = t.__class__',
    'x(n,t) = [n][0]', 'x(n,t) = x(n,t)', 'x(n,t) = sin(t, bad=1)',
    'x(n,t) = "bad"', 'x(t) = t',
])
def test_expression_language_rejects_python_code_and_invalid_definitions(expression):
    with pytest.raises(ValueError):
        compile_expressions((expression,))


def test_yaml_case_patterns_override_inherited_pattern_and_preserve_snapshot():
    config = load_config()
    config['control']['pattern'] = 'air-01'
    validate_config(config)
    config['comparison'] = {'cases': [{'name': 'tail', 'control': {'pattern': 'air-04'}}]}
    case = cases_from_config(config)[0][1]
    assert case['control']['resolved_pattern'] == 'air-04'
    np.testing.assert_array_equal(targets(1, case['control'])[:2], 0)
    saved = deepcopy(case)
    saved['control']['expressions'] = ['x(n,t) = 12']
    validate_config(saved)
    np.testing.assert_allclose(np.rad2deg(targets(1, saved['control'])), 12)


def test_pattern_comparison_keeps_physics_and_clocks_identical():
    config = load_config()
    original = deepcopy(config)
    cases = cases_from_config(config, motion_patterns=['air-01', 'air-04', 'air-10'])
    comparison = Comparison(cases)
    assert config == original
    for sim in comparison.simulations:
        assert sim.config['physics'] == config['physics']
        assert sim.config['balloon'] == config['balloon']
    for _ in range(5):
        comparison.step()
    assert all(sim.data.time == comparison.data.time for sim in comparison.simulations)
    assert not np.allclose(comparison.simulations[0].data.ctrl, comparison.simulations[1].data.ctrl)


def test_cli_comparison_saves_pattern_formulas_and_different_joint_commands(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['main', '--headless', '--compare-patterns', 'air-01', 'air-04',
                                   '--fluid-model', 'ellipsoid', '--duration', '.004', '--output', str(tmp_path)])
    main()
    manifest = yaml.safe_load((tmp_path / 'comparison.yaml').read_text())
    assert [case['pattern'] for case in manifest['cases']] == ['air-01', 'air-04']
    for entry in manifest['cases']:
        config = yaml.safe_load((tmp_path / entry['config']).read_text())
        assert config['control']['expressions'] == patterns()[entry['pattern']]['expressions']
        assert config['physics']['fluid_model'] == 'ellipsoid'
        with (tmp_path / entry['csv']).open() as file:
            assert len(list(csv.DictReader(file))) == 3


def test_list_patterns_exits_without_creating_output(tmp_path, monkeypatch, capsys):
    output = tmp_path / 'unused'
    monkeypatch.setattr('sys.argv', ['main', '--list-patterns', '--output', str(output)])
    main()
    assert 'air-10' in capsys.readouterr().out
    assert not output.exists()


def test_custom_yaml_expression_pattern():
    config = load_config()
    config['control'].update(mode='expressions', period=None, expressions=['A = 10', 'x(n,t) = A*n/4*sin(t)'])
    validate_config(config)
    np.testing.assert_allclose(np.rad2deg(targets(np.pi/2, config['control'])), [2.5, 5, 7.5, 10])


@pytest.mark.parametrize('name', ['air-02', 'wave-01', 'paddle'])
def test_motion_scale_changes_amplitude_without_changing_timing(name):
    control = load_config()['control']
    apply_pattern(control, name)
    for t in (0, .8, 3.1, 4.2, 7.6, 10):
        expected = targets(t, control)
        scaled = dict(control, motion_scale=2.)
        np.testing.assert_allclose(targets(t, scaled), np.clip(2 * expected, -np.pi/2, np.pi/2))
        np.testing.assert_array_equal(targets(t, dict(control, motion_scale=0)), 0)


@pytest.mark.parametrize('scale', [-1, float('nan'), float('inf')])
def test_invalid_motion_scale_is_rejected(scale):
    config = load_config()
    config['control']['motion_scale'] = scale
    with pytest.raises(ValueError, match='motion_scale'):
        validate_config(config)


def test_motion_scale_cli_saved_in_config(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['main', '--headless', '--neutral', '--pattern', 'air-02',
                                   '--motion-scale', '3', '--duration', '.004', '--output', str(tmp_path)])
    main()
    config = yaml.safe_load((tmp_path / 'config.yaml').read_text())
    assert config['control']['motion_scale'] == 3
    assert config['control']['pattern'] == 'air-02'


@pytest.mark.parametrize('name', list(patterns()))
@pytest.mark.parametrize('period', [2., 5., 8.])
def test_common_period_preserves_complete_wave_and_relative_stroke_timing(name, period):
    native = load_config()['control']
    native['period'] = None
    apply_pattern(native, name)
    normalized = dict(native, period=period)
    original_period = patterns()[name]['period']
    original_delay = patterns()[name].get('start_delay', 0.)
    for phase in (0, .1, .23, .5, .79, .99, 1.3):
        expected = targets(original_delay + phase * original_period, native)
        np.testing.assert_allclose(targets(phase * period, normalized), expected, atol=1e-12)
        np.testing.assert_allclose(targets(phase * period, normalized),
                                   targets((phase + 1) * period, normalized), atol=1e-12)


@pytest.mark.parametrize('mode', ['sine', 'traveling', 'asymmetric'])
def test_common_period_applies_to_legacy_modes(mode):
    control = load_config()['control']
    control.update(mode=mode, period=5, frequency=9)
    reference = dict(control, period=5, frequency=.2)
    for t in (0, .3, 1.6, 4.9):
        np.testing.assert_allclose(targets(t, control), targets(t, reference))
        np.testing.assert_allclose(targets(t, control), targets(t + 5, control), atol=1e-12)


@pytest.mark.parametrize('period', [0, -1, float('nan'), float('inf')])
def test_invalid_periods_are_rejected(period):
    config = load_config()
    config['control']['period'] = period
    with pytest.raises(ValueError, match='control.period'):
        validate_config(config)


def test_custom_expression_timing_metadata_is_required_for_common_period():
    config = load_config()
    config['control'].update(mode='expressions', expressions=['x(n,t) = 10*sin(t)'])
    with pytest.raises(ValueError, match='expression_period'):
        validate_config(config)
    config['control']['expression_period'] = 2 * np.pi
    validate_config(config)
    np.testing.assert_allclose(np.rad2deg(targets(.5, config['control'])), 10)


def test_period_cli_overrides_every_case_and_snapshots_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['main', '--headless', '--compare-patterns', 'air-06', 'paddle', 'wave-01',
                                   '--period', '8', '--duration', '.004', '--output', str(tmp_path)])
    main()
    manifest = yaml.safe_load((tmp_path / 'comparison.yaml').read_text())
    for case in manifest['cases']:
        config = load_config(tmp_path / case['config'])
        assert config['control']['period'] == 8
        for t in (.3, 1.5, 6.9):
            np.testing.assert_allclose(targets(t, config['control']), targets(t + 8, config['control']), atol=1e-12)


@pytest.mark.parametrize('name', list(patterns()))
def test_cycles_return_to_original_formula_start_without_added_envelope(name):
    control = load_config()['control']
    apply_pattern(control, name)
    initial = targets(0, control)
    for t in (2, 4, 6):
        np.testing.assert_allclose(targets(t, control), initial, atol=1e-12)
    if name in ('air-01', 'air-04', 'wave-01'):
        assert np.max(np.abs(initial)) > .1  # The stored wave starts bent; no forced straightening.


def test_air06_needs_both_signed_strokes_to_repeat():
    control = load_config()['control']
    apply_pattern(control, 'air-06')
    np.testing.assert_allclose(np.rad2deg(targets(.375, control)), [5, 10, 15, 20])
    np.testing.assert_allclose(targets(1, control), 0, atol=1e-12)
    np.testing.assert_allclose(np.rad2deg(targets(1.375, control)), [-5, -10, -15, -20])
    np.testing.assert_allclose(targets(2, control), 0, atol=1e-12)
    assert control['expression_period'] == 16


def test_previous_half_cycle_metadata_is_repaired_for_unchanged_air06_formula():
    config = load_config()
    apply_pattern(config['control'], 'air-06')
    config['control'].update(expression_period=8, expression_repeat_cycles=2)
    validate_config(config)
    assert config['control']['expression_period'] == 16
    assert 'expression_repeat_cycles' not in config['control']


@pytest.mark.parametrize('name,period,delay', [
    *[(f'air-{n:02}', 16 if n == 6 else 8, 0) for n in range(1, 11)],
    ('slow-sweep', 8, 0), ('paddle', 5, 0), ('wave-01', 2, 0),
])
def test_stored_period_matches_raw_expression_and_half_period_does_not(name, period, delay):
    pattern = patterns()[name]
    assert pattern['period'] == period
    output = compile_expressions(tuple(pattern['expressions']))
    samples = np.linspace(.013, period * 3, 601) + delay
    original = np.array([[output(n, t) for n in range(1, 5)] for t in samples])
    repeated = np.array([[output(n, t + period) for n in range(1, 5)] for t in samples])
    halfway = np.array([[output(n, t + period / 2) for n in range(1, 5)] for t in samples])
    np.testing.assert_allclose(repeated, original, atol=1e-9)
    assert np.max(np.abs(halfway - original)) > 1


def test_old_saved_expression_configs_keep_native_timing_without_period():
    config = load_config()
    apply_pattern(config['control'], 'paddle')
    # Explicit historical snapshot: new registrations have no startup wait.
    config['control']['expressions'].insert(4, 'tau(t) = max(0, t - 3)')
    config['control']['expressions'][5] = 'u(t) = tau(t)/T - floor(tau(t)/T)'
    config['control'].pop('period')
    config['control'].pop('expression_period')
    config['control'].pop('expression_start_delay')
    validate_config(config)
    np.testing.assert_array_equal(targets(2.5, config['control']), 0)
    np.testing.assert_allclose(np.rad2deg(targets(4, config['control'])), -45)


@pytest.mark.parametrize('period,peak_time,end_time', [(None, 1., 5.), (2., .4, 2.), (8., 1.6, 8.)])
def test_paddle_has_no_startup_wait_at_native_or_common_period(period, peak_time, end_time):
    control = load_config()['control']
    control['period'] = period
    apply_pattern(control, 'paddle')
    assert control['expression_start_delay'] == 0
    np.testing.assert_array_equal(targets(0, control), 0)
    assert np.all(targets(.05, control) < 0)
    np.testing.assert_allclose(np.rad2deg(targets(peak_time, control)), -45)
    np.testing.assert_allclose(targets(end_time, control), 0, atol=1e-12)
