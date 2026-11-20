"""Generate MJCF with explicit composite inertia and separate fluid geometry."""
from pathlib import Path
from copy import deepcopy
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .mesh_asset import prepare_obj
from .motion_patterns import apply_pattern, expression_targets, patterns

ROOT = Path(__file__).resolve().parents[1]


def merge_config(base, overrides):
    result = deepcopy(base)
    for key, value in overrides.items():
        result[key] = (merge_config(result[key], value)
                       if isinstance(value, dict) and isinstance(result.get(key), dict) else deepcopy(value))
    return result


def load_config(path=ROOT / 'config/default.yaml', _parents=None):
    path = Path(path).expanduser().resolve()
    parents = set() if _parents is None else _parents
    if path in parents:
        raise ValueError(f'Circular config extends: {path}')
    with open(path, encoding='utf-8') as f:
        c = yaml.safe_load(f)
    if not isinstance(c, dict):
        raise ValueError('Config must be a YAML mapping')
    if 'extends' in c:
        parent = path.parent / c.pop('extends')
        c = merge_config(load_config(parent, parents | {path}), c)
    return validate_config(c)


def validate_config(c):
    b, j, p, s, ctrl = (c[k] for k in ('balloon', 'joint', 'physics', 'simulation', 'control'))
    scale = ctrl.get('motion_scale', 1.)
    if not np.isfinite(scale) or scale < 0:
        raise ValueError('control.motion_scale must be finite and nonnegative')
    period = ctrl.get('period')
    if period is not None and (not np.isfinite(period) or period <= 0):
        raise ValueError('control.period must be finite and positive')
    if ctrl.get('pattern'):
        # Keep embedded formulas in saved results so runs remain reproducible
        # even when the registry is subsequently edited.
        if (ctrl.get('resolved_pattern') != ctrl['pattern']
                or ctrl.get('mode') != 'expressions' or 'expressions' not in ctrl):
            apply_pattern(ctrl, ctrl['pattern'])
    if ctrl['mode'] == 'expressions':
        # Old saved configs embed formulas without timing metadata. Fill only
        # missing metadata; never replace their saved formulas or common period.
        if ctrl.get('pattern') in patterns():
            pattern = patterns()[ctrl['pattern']]
            # Repair the previous half-cycle interpretation only for an
            # unchanged saved air-06 formula carrying that interpretation.
            if (ctrl.get('expression_repeat_cycles') == 2
                    and ctrl.get('expression_period') == 8
                    and ctrl['pattern'] == 'air-06'
                    and ctrl['expressions'] == pattern['expressions']):
                ctrl['expression_period'] = pattern['period']
            ctrl.pop('expression_repeat_cycles', None)
            ctrl.setdefault('expression_period', pattern['period'])
            ctrl.setdefault('expression_start_delay', pattern.get('start_delay', 0.))
        if period is not None:
            original_period = ctrl.get('expression_period')
            if original_period is None or not np.isfinite(original_period) or original_period <= 0:
                raise ValueError('control.expression_period must specify the positive original cycle length')
            delay = ctrl.get('expression_start_delay', 0.)
            if not np.isfinite(delay) or delay < 0:
                raise ValueError('control.expression_start_delay must be finite and nonnegative')
        expression_targets(0, ctrl)
    for value in [*b['dimensions'], b['volume'], b['film_mass'], j['radius'], j['length'],
                  j['torque_limit'], s['timestep'], s['duration']]:
        if not np.isfinite(value) or value <= 0:
            raise ValueError('Dimensions, mass, volume, time and torque limit must be positive')
    for value in [b['helium_density'], j['mass'], j['gap'], j['damping'], j['kp'], j['kv'],
                  p['air_density'], p['viscosity'], s['gravity'], ctrl['frequency']]:
        if not np.isfinite(value) or value < 0:
            raise ValueError('Physical coefficients must be finite and nonnegative')
    for value, size in [(b['dimensions'], 3), (b['buoyancy_center'], 3),
                        (p['ballast_position'], 3), (p['wind'], 3),
                        (p['fluid_coefficients'], 5), (ctrl['fixed_deg'], 4), (ctrl['phases_deg'], 4)]:
        if len(value) != size or not np.all(np.isfinite(value)):
            raise ValueError(f'Expected {size} finite values')
    if b['inertia_model'] != 'solid_ellipsoid':
        raise ValueError('Only solid_ellipsoid inertia is currently implemented')
    if p.get('fluid_model', 'ellipsoid') not in ('ellipsoid', 'mesh_surface'):
        raise ValueError('Unknown physics.fluid_model')
    drag = p.get('surface_drag_coefficient', 1.0)
    if not np.isfinite(drag) or drag < 0:
        raise ValueError('physics.surface_drag_coefficient must be finite and nonnegative')
    if ctrl['mode'] not in ('fixed', 'sine', 'traveling', 'asymmetric', 'expressions'):
        raise ValueError('Unknown control mode')
    if not np.isfinite(ctrl['phase_step_deg']):
        raise ValueError('control.phase_step_deg must be finite')
    if not np.isfinite(ctrl['amplitude_deg']):
        raise ValueError('control.amplitude_deg must be finite')
    if not np.isfinite(s['initial_height']):
        raise ValueError('simulation.initial_height must be finite')
    if np.any(np.asarray(p['fluid_coefficients']) < 0):
        raise ValueError('physics.fluid_coefficients must be nonnegative')
    if not 0 < ctrl['bend_fraction'] < 1:
        raise ValueError('bend_fraction must be between 0 and 1')
    return c


def fmt(values):
    return ' '.join(f'{x:.12g}' for x in np.atleast_1d(values))


def mesh(path, dimensions):
    # Elliptical capsule: elliptical cross section, rounded ends along X.
    length, width, height = dimensions
    cap = min(width / 2, length / 2)
    straight = length / 2 - cap
    vertices = []
    rings, slices = 25, 32
    for i in range(rings):
        theta = np.pi * (i + 0.001) / (rings - 1 + 0.002)
        x = cap * np.cos(theta) + (straight if np.cos(theta) >= 0 else -straight)
        for k in range(slices):
            phi = 2 * np.pi * k / slices
            vertices.append((x, width / 2 * np.sin(theta) * np.cos(phi),
                             height / 2 * np.sin(theta) * np.sin(phi)))
    lines = ['v ' + fmt(v) for v in vertices]
    for i in range(rings - 1):
        for k in range(slices):
            a, b = i * slices + k + 1, i * slices + (k + 1) % slices + 1
            d, e = a + slices, b + slices
            lines += [f'f {a} {d} {b}', f'f {b} {d} {e}']
    for k in range(1, slices - 1):
        lines += [f'f 1 {k+1} {k+2}',
                  f'f {(rings-1)*slices+1} {(rings-1)*slices+k+2} {(rings-1)*slices+k+1}']
    path.write_text('\n'.join(lines) + '\n')


def composite(parts):
    mass = sum(m for m, _, _ in parts)
    center = sum(m * r for m, r, _ in parts) / mass
    inertia = np.zeros((3, 3))
    for m, r, tensor in parts:
        d = r - center
        inertia += tensor + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, center, inertia


def build_model(c, directory=None):
    directory = Path(directory) if directory else ROOT / 'models'
    directory.mkdir(parents=True, exist_ok=True)
    b, j, p, s = (c[k] for k in ('balloon', 'joint', 'physics', 'simulation'))
    surface_drag = p.get('fluid_model', 'ellipsoid') == 'mesh_surface'
    mesh_attributes = {'name': 'balloon_mesh'}
    if b.get('mesh_file'):
        source = Path(b['mesh_file']).expanduser()
        if not source.is_absolute():
            source = ROOT / source
        asset = prepare_obj(source, directory / 'custom_balloon.obj', b)
        mesh_attributes['file'] = asset.name
    else:
        asset = directory / 'balloon.obj'
        mesh(asset, b['dimensions'])
        mesh_attributes['file'] = asset.name
    axes = np.array(b['dimensions']) / 2
    spacing = b['dimensions'][0] + j['gap']
    mass_b = b['film_mass'] + b['helium_density'] * b['volume']
    ballast = max(0, 5*p['air_density']*b['volume'] - 5*mass_b - 4*j['mass']) / 5 if p['neutral'] else 0
    if p['neutral'] and 5*p['air_density']*b['volume'] < 5*mass_b + 4*j['mass']:
        raise ValueError('Neutral buoyancy cannot be achieved by adding ballast')
    root = ET.Element('mujoco', model='five_balloon_robot')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep=str(s['timestep']), gravity=fmt([0,0,-s['gravity']]),
                  # Surface pressure is applied separately; retain only native
                  # viscous forces to avoid counting quadratic drag twice.
                  density=str(p['air_density'] if p['fluid'] and not surface_drag else 0),
                  viscosity=str(p['viscosity'] if p['fluid'] else 0), wind=fmt(p['wind']))
    assetnode = ET.SubElement(root, 'asset')
    ET.SubElement(assetnode, 'mesh', **mesh_attributes)
    ET.SubElement(assetnode, 'texture', name='ground_checker', type='2d', builtin='checker',
                  rgb1='0.3 0.35 0.4', rgb2='0.65 0.7 0.75', width='512', height='512')
    ET.SubElement(assetnode, 'material', name='ground_material', texture='ground_checker',
                  texrepeat='1 1', texuniform='true', reflectance='0.1')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -3 8', dir='0 0 -1')
    # Visual reference only: keep free-flight dynamics and collision settings.
    ET.SubElement(world, 'geom', name='ground', type='plane', pos='0 0 0', size='0 0 0.1',
                  material='ground_material', contype='0', conaffinity='0')
    parent = world
    for i in range(5):
        body = ET.SubElement(parent, 'body', name=f'balloon_{i+1}',
                             pos=fmt([0,0,s['initial_height']] if i == 0 else [spacing,0,0]))
        if i == 0:
            ET.SubElement(body, 'freejoint', name='root')
        else:
            ET.SubElement(body, 'joint', name=f'joint_{i}', type='hinge', axis='0 1 0',
                          pos=fmt([-spacing/2,0,0]), range=fmt([-np.pi/2,np.pi/2]), damping=str(j['damping']))
        tensor = np.diag(mass_b/5 * (np.sum(axes**2) - axes**2))
        parts = [(mass_b, np.zeros(3), tensor)]
        if i:
            r, h, m = j['radius'], j['length'], j['mass']
            jt = np.diag([m*(3*r*r+h*h)/12, m*r*r/2, m*(3*r*r+h*h)/12])
            parts.append((m, np.array([-spacing/2,0,0]), jt))
            ET.SubElement(body, 'geom', type='cylinder', size=fmt([r,h/2]),
                          pos=fmt([-spacing/2,0,0]), quat='0.70710678 0.70710678 0 0',
                          contype='0', conaffinity='0', rgba='0.2 0.2 0.2 1')
        if ballast:
            parts.append((ballast, np.array(p['ballast_position']), np.zeros((3,3))))
            ET.SubElement(body, 'geom', type='sphere', size='0.012', pos=fmt(p['ballast_position']),
                          contype='0', conaffinity='0', rgba='0.1 0.1 0.1 1')
        mass, center, inertia = composite(parts)
        ET.SubElement(body, 'inertial', mass=str(mass), pos=fmt(center),
                      fullinertia=fmt([inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]))
        ET.SubElement(body, 'geom', type='mesh', mesh='balloon_mesh', contype='0', conaffinity='0',
                      rgba=fmt([0.25+0.12*i,0.65,0.9-0.12*i,0.85]))
        fluid_shape = {'type': 'mesh', 'mesh': 'balloon_mesh'} if surface_drag else {'type': 'ellipsoid', 'size': fmt(axes)}
        ET.SubElement(body, 'geom', name=f'fluid_{i+1}', **fluid_shape,
                      contype='0', conaffinity='0', rgba='1 0.2 0.05 0.4', group='3',
                      fluidshape='ellipsoid', fluidcoef=fmt([0]*5 if surface_drag else p['fluid_coefficients']))
        ET.SubElement(body, 'site', name=f'buoyancy_{i+1}', pos=fmt(b['buoyancy_center']),
                      size='0.008', rgba='1 0.3 0.1 1', group='4')
        parent = body
    actuators = ET.SubElement(root, 'actuator')
    for i in range(1,5):
        ET.SubElement(actuators, 'position', name=f'servo_{i}', joint=f'joint_{i}',
                      kp=str(j['kp']), kv=str(j['kv']), ctrllimited='true', ctrlrange=fmt([-np.pi/2,np.pi/2]),
                      forcelimited='true', forcerange=fmt([-j['torque_limit'],j['torque_limit']]))
    ET.indent(root)
    path = directory / 'balloon_robot.xml'
    ET.ElementTree(root).write(path, encoding='unicode')
    return path
