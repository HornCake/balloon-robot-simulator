import csv
import mujoco
import numpy as np


class Logger:
    def __init__(self, path):
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        columns = ['time']
        columns += [f'position_{a}' for a in 'xyz'] + [f'quaternion_{a}' for a in 'wxyz']
        columns += [f'velocity_{a}' for a in 'xyz'] + [f'angular_velocity_{a}' for a in 'xyz']
        columns += [f'balloon_{i}_com_{a}' for i in range(1,6) for a in 'xyz']
        columns += [f'joint_{i}_{quantity}' for quantity in ('target','angle','velocity','torque') for i in range(1,5)]
        columns += [f'buoyancy_{i}' for i in range(1,6)] + [f'com_{a}' for a in 'xyz']
        self.writer.writerow(columns)

    def record(self, sim):
        d = sim.data
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(sim.model, d, mujoco.mjtObj.mjOBJ_BODY, sim.body_ids[0], velocity, 0)
        self.writer.writerow(np.concatenate(([d.time], d.qpos[:7], velocity[3:], velocity[:3],
                                            d.xipos[sim.body_ids].ravel(), d.ctrl, d.qpos[7:], d.qvel[6:],
                                            d.actuator_force, sim.forces, sim.center_of_mass)))

    def close(self):
        self.file.close()
