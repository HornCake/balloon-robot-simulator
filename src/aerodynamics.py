"""Quasi-steady, windward surface-pressure drag; not a CFD solver."""
import mujoco
import numpy as np


class MeshSurfaceDrag:
    def __init__(self, model, config):
        self.model = model
        self.config = config
        self.geoms = [model.geom(f'fluid_{i}').id for i in range(1, 6)]
        self.panels = {}
        for geom in self.geoms:
            mesh = int(model.geom_dataid[geom])
            if mesh in self.panels:
                continue
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start:start + count].astype(float)
            start, count = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
            faces = model.mesh_face[start:start + count]
            # A closed, consistently wound surface is needed for outward normals.
            edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
            _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
            _, directed_counts = np.unique(edges, axis=0, return_counts=True)
            if np.any(counts != 2) or np.any(directed_counts != 1):
                raise ValueError('Surface drag requires a closed mesh with consistent face winding')
            triangles = vertices[faces]
            centers = triangles.mean(axis=1)
            area_vectors = np.cross(triangles[:, 1] - triangles[:, 0],
                                    triangles[:, 2] - triangles[:, 0]) / 2
            volume = np.einsum('ij,ij->', triangles[:, 0], area_vectors) / 3
            if abs(volume) < 1e-12:
                raise ValueError('Surface drag requires a mesh with nonzero enclosed volume')
            area_vectors *= np.sign(volume)
            areas = np.linalg.norm(area_vectors, axis=1)
            valid = areas > 1e-14
            centers, areas = centers[valid], areas[valid]
            normals = area_vectors[valid] / areas[:, None]
            # (angular velocity x center) dot normal = angular velocity dot
            # (center x normal). The same lever also maps pressure to torque.
            levers = np.cross(centers, normals)
            self.panels[mesh] = (normals, levers, areas)

    def apply(self, data):
        p = self.config['physics']
        if not p['fluid']:
            return
        coefficient = p.get('surface_drag_coefficient', 1.0)
        wind = np.asarray(p['wind'])
        velocity = np.empty(6)
        for geom in self.geoms:
            normals, levers, areas = self.panels[int(self.model.geom_dataid[geom])]
            rotation = data.geom_xmat[geom].reshape(3, 3)
            mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_GEOM,
                                    geom, velocity, 0)
            angular = rotation.T @ velocity[:3]
            linear = rotation.T @ (velocity[3:] - wind)
            outward_speed = np.maximum(normals @ linear + levers @ angular, 0)
            # Pressure only on faces advancing into the air. This force always
            # dissipates energy relative to the wind, including during rotation.
            pressure = -0.5 * p['air_density'] * coefficient * areas * outward_speed**2
            force = rotation @ (pressure @ normals)
            torque = rotation @ (pressure @ levers)
            mujoco.mj_applyFT(self.model, data, force, torque, data.geom_xpos[geom],
                             int(self.model.geom_bodyid[geom]), data.qfrc_applied)
