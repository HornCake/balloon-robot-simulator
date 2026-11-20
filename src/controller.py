"""Independent waveforms; angles returned in radians."""
import numpy as np
from .motion_patterns import expression_targets


def fixed(t, c):
    return np.deg2rad(c['fixed_deg'])


def frequency(c):
    return 1 / c['period'] if c.get('period') is not None else c['frequency']


def sine(t, c):
    return np.deg2rad(c['amplitude_deg']) * np.sin(
        2 * np.pi * frequency(c) * t + np.deg2rad(c['phases_deg']))


def traveling(t, c):
    return np.deg2rad(c['amplitude_deg']) * np.sin(
        2 * np.pi * frequency(c) * t - np.arange(4) * np.deg2rad(c['phase_step_deg']))


def asymmetric(t, c):
    phase = (frequency(c) * t - np.arange(4) * c['phase_step_deg'] / 360) % 1
    fraction = c['bend_fraction']
    wave = np.where(phase < fraction, -np.cos(np.pi * phase / fraction),
                    np.cos(np.pi * (phase - fraction) / (1 - fraction)))
    return np.deg2rad(c['amplitude_deg']) * wave


MODES = dict(fixed=fixed, sine=sine, traveling=traveling, asymmetric=asymmetric,
             expressions=expression_targets)


def targets(t, config):
    return np.clip(MODES[config['mode']](t, config) * config.get('motion_scale', 1.),
                   -np.pi / 2, np.pi / 2)
