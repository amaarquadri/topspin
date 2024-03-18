"""
Usage:
python run_guide_mode.py
Commands:
1-> Print Tool Pose
2-> Print Joint Pose
3-> Stop Skill and Exit
4-> Stop Skill and Reset to Home Position
"""
import numpy as np
import time
import sys
from frankapy import FrankaArm
if __name__ == "__main__":
    
    start = time.time()
    fa = FrankaArm(with_gripper=False)
    fa.stop_skill()
    fa.reset_joints()
	
    j = fa.get_joints()
    # j = [-4.83250412e-03, -9.17956899e-01,  1.03874710e-01, -2.58069315e+00,
    #     1.97509313e-03,  2.82994042e+00,  7.81663783e-01]
    
    pose = fa.get_pose()
    A = 0.05
    w = 2.0
    t0 = time.perf_counter()
    dt = 0.1
    # sending sine commands to ee
    while(True):
        t = time.perf_counter() - t0
        delta = A * np.sin(w * t)
        global_j = j.copy()
        global_j[3] += delta
        fa.goto_joints(global_j, dynamic=True, buffer_time=10)
        print(delta)

        # time.sleep(dt)


    print("done")
