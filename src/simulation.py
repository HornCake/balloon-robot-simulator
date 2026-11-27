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
        # Track the outermost +X vertex of the last balloon in body coordinates.
        tip_body = self.body_ids[-1]
        geom = next(i for i in range(self.model.ngeom)
                    if self.model.geom_bodyid[i] == tip_body
                    and self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH
                    and self.model.geom_group[i] != 3)
        mesh = self.model.geom_dataid[geom]
        first = self.model.mesh_vertadr[mesh]
        vertices = self.model.mesh_vert[first:first + self.model.mesh_vertnum[mesh]]
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, self.model.geom_quat[geom])
        local = vertices @ rotation.reshape(3, 3).T + self.model.geom_pos[geom]
        self.local_tip = local[np.argmax(local[:, 0])]
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.refresh()
        self.initial_tip = self.tip_position
        self.initial_joint_angles = self.data.qpos[7:].copy()
        self.initial_center_of_mass = self.center_of_mass.copy()

    @property
    def tip_position(self):
        body = self.body_ids[-1]
        return self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ self.local_tip

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
