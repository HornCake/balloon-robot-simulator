import mujoco
import numpy as np


def apply(model, data, config):
    """Overwrite applied generalized forces; buoyancy acts at world-space sites."""
    data.qfrc_applied[:] = 0
    force = config['physics']['air_density'] * config['balloon']['volume'] * config['simulation']['gravity']
    force = force if config['physics']['buoyancy'] else 0.0
    for i in range(1, 6):
        body = model.body(f'balloon_{i}').id
        point = data.site_xpos[model.site(f'buoyancy_{i}').id]
        mujoco.mj_applyFT(model, data, np.array([0., 0., force]), np.zeros(3), point, body, data.qfrc_applied)
    return np.full(5, force)
