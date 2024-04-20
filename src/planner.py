from config import Config
from heuristics import Heuristics
import solver
import numpy as np

config = Config()
heuristics = Heuristics()
    
def get_plan(x, v, franka_arm):
    t, x_des, theta_des, v_des = heuristics.run(x, v)

    q_des = solver.solve(franka_arm, x_des, theta_des[0], theta_des[1])
    hit_offset = np.zeros((7,))
    hit_offset[config.control_joint] = config.swing_angle
    q_pre = q_des - hit_offset

    J = franka_arm.get_jacobian(q_des)
    w = J[2, config.control_joint]
    e_dot_des = v_des[2] / w

    return t, q_des, q_pre, e_dot_des