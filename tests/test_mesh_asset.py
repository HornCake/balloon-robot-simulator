import numpy as np
import pytest

from src.mesh_asset import prepare_obj
from src.model import ROOT


def read_vertices(path):
    return np.array([[float(x) for x in line.split()[1:4]]
                     for line in path.read_text().splitlines() if line.startswith('v ')])


def test_import_rotation_center_and_fit_preserve_source_and_normals(tmp_path):
    source = tmp_path / 'offset.obj'
    # A translated tetrahedron with a normal; source long direction is Z.
    source.write_text('v 10 20 30\nv 12 20 30\nv 10 24 30\nv 10 20 38\n'
                      'vn 0 0 1\nf 1//1 3//1 2//1\nf 1 2 4\nf 2 3 4\nf 3 1 4\n')
    original = source.read_bytes()
    prepared = prepare_obj(source, tmp_path / 'prepared.obj', {
        'mesh_scale': [.001, .001, .001], 'mesh_rotation_deg': [0, 90, 0],
        'mesh_center': True, 'mesh_fit_dimensions': True, 'dimensions': [.65, .45, .3]})
    vertices = read_vertices(prepared)
    np.testing.assert_allclose(np.ptp(vertices, axis=0), [.65, .45, .3])
    np.testing.assert_allclose((vertices.max(0) + vertices.min(0)) / 2, 0, atol=1e-12)
    normal = next(line for line in prepared.read_text().splitlines() if line.startswith('vn '))
    np.testing.assert_allclose(list(map(float, normal.split()[1:])), [1, 0, 0], atol=1e-12)
    assert source.read_bytes() == original


def test_import_never_overwrites_source(tmp_path):
    source = tmp_path / 'custom_balloon.obj'
    source.write_bytes((ROOT / 'assets/balloon_87.obj').read_bytes())
    original = source.read_bytes()
    prepared = prepare_obj(source, source, {'mesh_rotation_deg': [0, 0, 90]})
    assert prepared != source
    assert source.read_bytes() == original


def test_import_rejects_untriangulated_mesh(tmp_path):
    source = tmp_path / 'quad.obj'
    source.write_text('v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n')
    with pytest.raises(ValueError, match='Triangulate'):
        prepare_obj(source, tmp_path / 'out.obj', {})
