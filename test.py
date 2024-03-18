import numpy as np
import time
import sys
from frankapy import FrankaArm
from autolab_core import RigidTransform

if __name__ == "__main__":
    
    start = time.time()
    print("T0")
    fa = FrankaArm()
    print(">>> Creating object : ", fa.get_pose().translation)
    fa.reset_joints()
    print("T1")
    fa.open_gripper()
    print("T2")
    fa.run_guide_mode(10000,block=False)

    T_ee_world = fa.get_pose()
    joints = fa.get_joints()

    print(">>> Creating object : ", fa.get_pose().translation)
