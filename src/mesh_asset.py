"""Prepare reusable OBJ assets without modifying the source file."""
from pathlib import Path

import numpy as np


def vector(value, name, positive=False):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)) or (positive and np.any(result <= 0)):
        raise ValueError(f'balloon.{name} must contain 3 finite' + (' positive values' if positive else ' values'))
    return result


def prepare_obj(source, destination, balloon):
    """Scale, rotate about fixed X/Y/Z axes, center, then optionally fit bounds."""
    source, destination = Path(source), Path(destination)
    if source.suffix.lower() != '.obj':
        raise ValueError('balloon.mesh_file must be an OBJ file')
    if source.resolve() == destination.resolve():
        destination = destination.with_name('prepared_balloon.obj')
    lines = source.read_text().splitlines()
    vertices = np.array([[float(x) for x in line.split()[1:4]]
                         for line in lines if line.startswith('v ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.all(np.isfinite(vertices)):
        raise ValueError('OBJ must contain finite 3D vertices')
    if not any(line.startswith('f ') for line in lines):
        raise ValueError('OBJ must contain triangular faces')
    if any(len(line.split()) != 4 for line in lines if line.startswith('f ')):
        raise ValueError('Triangulate the OBJ faces before importing')
    scale = vector(balloon.get('mesh_scale', [1, 1, 1]), 'mesh_scale', positive=True)
    angles = np.deg2rad(vector(balloon.get('mesh_rotation_deg', [0, 0, 0]), 'mesh_rotation_deg'))
    x, y, z = angles
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    rotation = rz @ ry @ rx
    transformed = (vertices * scale) @ rotation.T
    fit = np.ones(3)
    if balloon.get('mesh_center', False):
        transformed -= (transformed.min(axis=0) + transformed.max(axis=0)) / 2
    if balloon.get('mesh_fit_dimensions', False):
        size = np.ptp(transformed, axis=0)
        if np.any(size <= 1e-12):
            raise ValueError('Cannot fit an OBJ with zero extent along an axis')
        fit = vector(balloon['dimensions'], 'dimensions', positive=True) / size
        transformed *= fit
    # Leave an untransformed OBJ byte-for-byte intact for existing configs.
    if (np.array_equal(scale, [1, 1, 1]) and not np.any(angles)
            and not balloon.get('mesh_center', False) and not balloon.get('mesh_fit_dimensions', False)):
        destination.write_bytes(source.read_bytes())
        return destination
    output = ['# Prepared for MuJoCo; source OBJ is unchanged.']
    index = 0
    for line in lines:
        if line.startswith('v '):
            line = 'v ' + ' '.join(f'{v:.12g}' for v in transformed[index])
            index += 1
        elif line.startswith('vn '):
            normal = (np.array([float(v) for v in line.split()[1:4]]) / scale) @ rotation.T / fit
            length = np.linalg.norm(normal)
            if length:
                normal /= length
            line = 'vn ' + ' '.join(f'{v:.12g}' for v in normal)
        output.append(line)
    destination.write_text('\n'.join(output) + '\n')
    return destination
