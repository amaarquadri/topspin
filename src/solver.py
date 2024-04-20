import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import root
from frankapy import FrankaArm
from frankapy import FrankaConstants
from constants import CONTROL_JOINT


def solve(franka_arm: FrankaArm,
          position: np.ndarray,
          roll: float, pitch: float) -> np.ndarray:
    def constraints(q: np.ndarray) -> np.ndarray:
        end_effector_tf = franka_arm.get_links_transforms(q)[-1]
        end_effector_position = end_effector_tf[:3, 3]
        end_effector_rpy = Rotation.from_matrix(end_effector_tf[:3, :3]).as_euler('ZYX')[::-1]

        position_error = end_effector_position - position
        rpy_error = end_effector_rpy - np.array([roll, pitch, 0])

        jac = franka_arm.get_jacobian(q)

        return np.concatenate((position_error, rpy_error[:2], jac[:2, CONTROL_JOINT]))

    q_guess = FrankaConstants.HOME_JOINTS
    q_solution = root(constraints, q_guess).x
    return q_solution
