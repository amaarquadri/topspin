import numpy as np
import time
import sys
from frankapy import FrankaArm
from autolab_core import RigidTransform

if __name__ == "__main__":
    
    start = time.time()
    fa = FrankaArm()
    # fa.reset_joints()
    fa.open_gripper()
    fa.run_guide_mode(10000,block=False)

    T_ee_world = fa.get_pose()
    joints = fa.get_joints()

    print(">>> Creating object : ", fa.get_pose().translation)

    del_pose= RigidTransform(rotation=np.array([
                                [1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]]),
                                translation=np.array([0.0, 0.0, 0.1]),
                                from_frame='franka_tool', to_frame='franka_tool')

    print(">>> Started : ", fa.get_pose().translation)

    fa.stop_skill()
    # fa.reset_joints()
    fa.goto_pose_delta(del_pose, use_impedance=False)

    print(">>> Completed : ". fa.get_pose().translation)