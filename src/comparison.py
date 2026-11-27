"""Independent simulation cases sharing a display-only MuJoCo scene."""
from copy import deepcopy
from pathlib import Path
import re
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

from .logger import Logger
from .model import add_viewer_lighting, build_model, load_config, validate_config, merge_config
from .simulation import Simulation
from .motion_patterns import apply_pattern


def cases_from_config(config, methods=None, config_paths=None, motion_patterns=None):
    if motion_patterns:
        cases = []
        for name in motion_patterns:
            case = deepcopy(config)
            case.pop('comparison', None)
            apply_pattern(case['control'], name)
            cases.append((name, validate_config(case)))
        return cases
    if config_paths:
        return [(Path(path).stem, load_config(path)) for path in config_paths]
    specifications = ([{'name': method, 'fluid_model': method} for method in methods]
                      if methods else config.get('comparison', {}).get('cases'))
    if specifications is None:
        specifications = [{'name': 'Ellipsoid', 'fluid_model': 'ellipsoid'},
                          {'name': 'Mesh surface', 'fluid_model': 'mesh_surface'}]
    if not specifications:
        raise ValueError('Comparison requires at least one case')
    cases = []
    for index, specification in enumerate(specifications, 1):
        if not isinstance(specification, dict):
            raise ValueError('Each comparison case must be a YAML mapping')
        case = merge_config(config, {key: value for key, value in specification.items()
                                     if key not in ('name', 'fluid_model')})
        case.pop('comparison', None)
        if 'fluid_model' in specification:
            case['physics']['fluid_model'] = specification['fluid_model']
        elif ('mesh_file' in specification.get('balloon', {})
              and 'fluid_model' not in specification.get('physics', {})):
            # Selecting a new mesh implies surface drag unless the case
            # explicitly asks for an ellipsoid approximation.
            case['physics']['fluid_model'] = 'mesh_surface'
        method = case['physics'].get('fluid_model', 'ellipsoid')
        if method not in ('ellipsoid', 'mesh_surface'):
            raise ValueError(f'Unknown fluid_model for comparison case {index}: {method}')
        cases.append((str(specification.get('name', f'Case {index}')), validate_config(case)))
    return cases


class Comparison:
    def case_color(self, index):
        return [(0.15, .6, 1.), (1., .45, .1), (.15, .8, .4),
                (.7, .4, 1.), (1., .7, .1), (1., .3, .6)][index % 6]

    def __init__(self, cases):
        if not cases:
            raise ValueError('Comparison requires at least one case')
        self.names = [name for name, _ in cases]
        self.simulations = [Simulation(deepcopy(config)) for _, config in cases]
        timestep = self.simulations[0].model.opt.timestep
        duration = self.simulations[0].config['simulation']['duration']
        if any(not np.isclose(sim.model.opt.timestep, timestep, rtol=0, atol=1e-12)
               or sim.config['simulation']['duration'] != duration for sim in self.simulations):
            raise ValueError('Comparison cases must have the same timestep and duration')
        self.config = deepcopy(self.simulations[0].config)
        widths = [sim.config['balloon']['dimensions'][1] for sim in self.simulations]
        gap = max(.6, .5 * max(widths))
        self.offsets = np.zeros((len(cases), 3))
        # Separate cases along Y only, leaving the propulsion coordinate X aligned.
        cursor = 0.
        for index, width in enumerate(widths):
            self.offsets[index, 1] = cursor + width / 2
            cursor += width + gap
        self.offsets[:, 1] -= (cursor - gap) / 2
        self.lane_bounds = (-(cursor - gap) / 2, (cursor - gap) / 2)
        self.initial_centers = np.array([sim.center_of_mass.copy() for sim in self.simulations])
        self.initial_target = (self.initial_centers + self.offsets).mean(axis=0)
        self.model = self._display_model(timestep)
        self.case_geom_ids = []
        self.case_site_ids = []
        for index in range(len(cases)):
            bodies = [self.model.body(f'case_{index}_balloon_{body}').id for body in range(1, 6)]
            self.case_geom_ids.append(np.flatnonzero(np.isin(self.model.geom_bodyid, bodies)))
            self.case_site_ids.append(np.flatnonzero(np.isin(self.model.site_bodyid, bodies)))
        self.data = mujoco.MjData(self.model)
        self.sync_display()

    def _display_model(self, timestep):
        root = ET.Element('mujoco', model='air_resistance_comparison_display')
        ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
        ET.SubElement(root, 'option', timestep=str(timestep), gravity='0 0 0', density='0', viscosity='0')
        add_viewer_lighting(root)
        assets = ET.SubElement(root, 'asset')
        world = ET.SubElement(root, 'worldbody')
        ET.SubElement(world, 'light', pos='0 -3 8', dir='0 0 -1')
        with tempfile.TemporaryDirectory(prefix='balloon-comparison-') as directory:
            for index, sim in enumerate(self.simulations):
                path = build_model(sim.config, Path(directory) / str(index))
                source = ET.parse(path).getroot()
                prefix = f'case_{index}_'
                asset_names = {node.attrib['name'] for node in source.find('asset') if 'name' in node.attrib}
                for node in source.find('asset'):
                    if index > 0 and node.tag == 'texture' and node.get('type') == 'skybox':
                        continue
                    node = deepcopy(node)
                    for element in node.iter():
                        for attribute in ('name', 'texture', 'material', 'mesh'):
                            if element.get(attribute) in asset_names:
                                element.set(attribute, prefix + element.get(attribute))
                        if element.get('file'):
                            element.set('file', str(path.parent / element.get('file')))
                    assets.append(node)
                for node in source.find('worldbody'):
                    if node.tag != 'body' and not (index == 0 and node.tag == 'geom'):
                        continue
                    node = deepcopy(node)
                    for element in node.iter():
                        if element.get('name'):
                            element.set('name', prefix + element.get('name'))
                        for attribute in ('mesh', 'material'):
                            if element.get(attribute):
                                element.set(attribute, prefix + element.get(attribute))
                    world.append(node)
            xml = Path(directory) / 'display.xml'
            ET.ElementTree(root).write(xml, encoding='unicode')
            return mujoco.MjModel.from_xml_path(str(xml))

    def sync_display(self):
        position, velocity = 0, 0
        for sim, offset in zip(self.simulations, self.offsets):
            self.data.qpos[position:position + sim.model.nq] = sim.data.qpos
            self.data.qpos[position:position + 3] += offset
            self.data.qvel[velocity:velocity + sim.model.nv] = sim.data.qvel
            position += sim.model.nq
            velocity += sim.model.nv
        self.data.time = self.simulations[0].data.time
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        for sim in self.simulations:
            sim.step()
        self.sync_display()

    def reset(self):
        for sim in self.simulations:
            sim.reset()
        self.sync_display()

    @property
    def centers(self):
        return np.array([sim.center_of_mass for sim in self.simulations]) + self.offsets

    @property
    def center_of_mass(self):
        return self.centers.mean(axis=0)

    @property
    def camera_distance(self):
        return self.view_distance(self.camera_target())

    def camera_target(self, follow=False):
        target = self.center_of_mass.copy() if follow else self.initial_target.copy()
        # Track rising balloons in Z while preserving the X/Y reference in overview.
        target[2] = self.center_of_mass[2]
        return target

    def _view_radii(self, target, focus=None):
        lower, upper = [], []
        for index, (sim, offset) in enumerate(zip(self.simulations, self.offsets)):
            if focus is not None and index != focus:
                continue
            rotation = sim.data.xmat[sim.body_ids].reshape(-1, 3, 3)
            extent = np.abs(rotation) @ (np.array(sim.config['balloon']['dimensions']) / 2)
            positions = sim.data.xpos[sim.body_ids] + offset
            lower.append((positions - extent).min(axis=0))
            upper.append((positions + extent).max(axis=0))
        return np.maximum(np.abs(np.min(lower, axis=0) - target),
                          np.abs(np.max(upper, axis=0) - target))

    def view_height(self, target):
        radius = self._view_radii(target)
        # Allow room for labels and the X direction marker in a 4:3 view.
        return max(3., 2.2 * max(radius[1] + .6, (radius[0] + .3) * .75))

    def view_distance(self, target):
        # Clearance above the highest case affects camera position, but not
        # the orthographic scale used to compare propulsion across heights.
        return max(4., self.view_height(target) * 1.25 + self._view_radii(target)[2])

    def labels(self, focus=None, x_direction='right', top=True):
        for index, (name, sim, center) in enumerate(zip(self.names, self.simulations, self.centers), 1):
            if focus is not None and index - 1 != focus:
                continue
            # MuJoCo label storage is limited to 100 bytes.
            short_name = name if len(name) <= 18 else name[:17] + '~'
            dx = sim.center_of_mass[0] - self.initial_centers[index - 1, 0]
            label = f'{index}: {short_name}  dx={dx:+.3f}m'.encode('utf-8')[:99].decode('utf-8', errors='ignore')
            width = 5 * sim.config['balloon']['dimensions'][0] + 4 * sim.config['joint']['gap']
            sign = 1 if x_direction == 'left' else -1
            offset = [sign * width / 2, -sign * (sim.config['balloon']['dimensions'][1] / 2 + .15), .2]
            if not top:
                offset[1] = 0
                offset[2] = sim.config['balloon']['dimensions'][2] / 2 + .3
            yield label, center + offset

    def draw_annotations(self, scene, clear=True, focus=None, x_direction='right', top=True, side_overlay=False):
        if clear:
            scene.ngeom = 0

        def geom(kind, position, color):
            if scene.ngeom >= scene.maxgeom:
                return None
            result = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(result, kind, np.zeros(3), np.asarray(position), np.eye(3).ravel(), np.asarray(color))
            scene.ngeom += 1
            return result

        for index, (label, position) in enumerate(self.labels(focus, x_direction, top)):
            color = [1., 1., 1., 1.]
            if side_overlay:
                # The Y lanes project onto the same XZ plane. Keep a separate,
                # color-matched legend above them, without shifting the robots.
                radius = self._view_radii(self.center_of_mass)
                sign = 1 if x_direction == 'left' else -1
                position = self.center_of_mass + [sign * radius[0], 0, radius[2] + .4 + .23 * index]
                color = [*self.case_color(index), 1.]
                color_name = ['blue', 'orange', 'green', 'purple', 'yellow', 'pink'][index % 6]
                label = f'[{color_name}] {label}'.encode('utf-8')[:99].decode('utf-8', errors='ignore')
            item = geom(mujoco.mjtGeom.mjGEOM_LABEL, position, color)
            if item is not None:
                item.label = label
        # Display-only X=0 reference and +X arrow. Orthographic overview keeps
        # them aligned with the robots even when the robots rise above the floor.
        y_min, y_max = self.lane_bounds
        arrow_y, arrow_z = y_min - .45, .02
        line_start, line_end = [0, y_min - .2, .02], [0, y_max + .2, .02]
        if (focus is not None or side_overlay) and not top:
            center = self.center_of_mass if focus is None else self.centers[focus]
            arrow_y = center[1]
            arrow_z = center[2] - .5
            line_start, line_end = [0, arrow_y, arrow_z], [0, arrow_y, arrow_z + 1.]
        for kind, start, end, width in [
            (mujoco.mjtGeom.mjGEOM_LINE, line_start, line_end, 2.),
            (mujoco.mjtGeom.mjGEOM_ARROW, [0, arrow_y, arrow_z], [1, arrow_y, arrow_z], .02),
        ]:
            item = geom(kind, start, [.15, .55, 1., 1.])
            if item is not None:
                mujoco.mjv_connector(item, kind, width, np.array(start), np.array(end))
        item = geom(mujoco.mjtGeom.mjGEOM_LABEL, [1.35 if x_direction == 'left' else 1.1, arrow_y, arrow_z], [.15, .55, 1., 1.])
        if item is not None:
            item.label = '+X'


class ComparisonLogger:
    def __init__(self, comparison, directory):
        self.paths = []
        self.loggers = []
        for index, (name, sim) in enumerate(zip(comparison.names, comparison.simulations), 1):
            slug = re.sub(r'[^a-zA-Z0-9_-]+', '_', name).strip('_') or 'case'
            folder = Path(directory) / f'{index:02d}_{slug}'
            folder.mkdir(parents=True, exist_ok=True)
            (folder / 'config.yaml').write_text(yaml.safe_dump(sim.config, sort_keys=False))
            self.paths.append(folder / 'trajectory.csv')
            self.loggers.append(Logger(self.paths[-1]))
        manifest = [{'name': name, 'fluid_model': sim.config['physics'].get('fluid_model', 'ellipsoid'),
                     'pattern': sim.config['control'].get('pattern'),
                     'csv': str(path.relative_to(directory)), 'config': str(path.parent.relative_to(directory) / 'config.yaml')}
                    for name, sim, path in zip(comparison.names, comparison.simulations, self.paths)]
        (Path(directory) / 'comparison.yaml').write_text(yaml.safe_dump({'cases': manifest}, sort_keys=False))

    def record(self, comparison):
        for logger, sim in zip(self.loggers, comparison.simulations):
            logger.record(sim)

    def close(self):
        for logger in self.loggers:
            logger.close()
