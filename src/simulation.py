import tempfile
import mujoco
import numpy as np
from .model import build_model
from .controller import targets
from .buoyancy import apply
from .aerodynamics import MeshSurfaceDrag


class Simulation:
    def __init__(self, config):
        self.config = config
        with tempfile.TemporaryDirectory(prefix='balloon-') as directory:
            self.model = mujoco.MjModel.from_xml_path(str(build_model(config, directory)))
        self.data = mujoco.MjData(self.model)
        self.body_ids = [self.model.body(f'balloon_{i}').id for i in range(1, 6)]
        self.surface_drag = (MeshSurfaceDrag(self.model, config)
                             if config['physics'].get('fluid_model', 'ellipsoid') == 'mesh_surface' else None)
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.refresh()
        self.initial_joint_angles = self.data.qpos[7:].copy()
        self.initial_center_of_mass = self.center_of_mass.copy()

    def refresh(self):
        mujoco.mj_forward(self.model, self.data)
        self.data.ctrl[:] = targets(self.data.time, self.config['control'])
        self.forces = apply(self.model, self.data, self.config)
        if self.surface_drag is not None:
            self.surface_drag.apply(self.data)
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        self.refresh()
        mujoco.mj_step(self.model, self.data)
        self.refresh()
        if not np.all(np.isfinite(self.data.qpos)) or np.any(self.data.warning.number):
            raise RuntimeError('MuJoCo numerical warning; reduce timestep or controller gains')

    @property
    def center_of_mass(self):
        masses = self.model.body_mass[self.body_ids]
        return np.average(self.data.xipos[self.body_ids], axis=0, weights=masses)

    @property
    def displacement(self):
        return self.center_of_mass - self.initial_center_of_mass

    @property
    def horizontal_distance(self):
        return float(np.linalg.norm(self.displacement[:2]))

    @property
    def initial_shape_error_deg(self):
        return float(np.rad2deg(np.max(np.abs(self.data.qpos[7:] - self.initial_joint_angles))))
