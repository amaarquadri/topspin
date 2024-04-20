from config import Config
from heuristics import Heuristics
import solver
import numpy as np

class Planner:
    def __init__(self, franka_arm):
        self.config = Config()
        self.arm = franka_arm

        self.heuristics = Heuristics()
    
    def get_plan(self, x, y, z, vx, vy, vz):
        t, x_des, y_des, z_des, r_des, p_des, yaw_des, vz_des = self.heuristics.run(x, y, z, vx, vy, vz)

        q_des = solver.solve(self.franka_arm, r_des, p_des)
        hit_offset = np.zeros((7,))
        hit_offset[self.control_joint] = self.config.swing_angle
        q_pre = q_des - hit_offset

        J = self.arm.get_jacobian(q_des)
        w = J[2, self.control_joint]
        e_dot_des = vz_des / w

        return t, q_des, q_pre, e_dot_des
    
    def execute_plan(self, t, q_des, q_pre):
        self.arm.goto_joints(q_pre)
        