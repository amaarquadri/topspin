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
    # fa.reset_joints()
	
    j = [-1.71721697,  0.93825313,  1.16560006, -2.42582475,  0.63423065,  1.98583073, 1.53964265]
    fa.goto_joints(j)
    
    # testing if racket holds in unstable pose
    # j[0] += 0.1
    # j[5] += 1.57
    # fa.goto_joints(j)
    # fa.reset_joints()
    # fa.goto_joints(j)

    # stream ping pong center pose
    fa.run_guide_mode(10000,block=False)
    while(True):
        print("--- Iter ---")
        print(fa.get_pose())
        print(fa.get_joints())
        time.sleep(0.2)

    print("done")
