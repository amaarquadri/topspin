import rospy
from geometry_msgs.msg import PoseStamped
import time
from frankapy import FrankaArm

from config import Config
import heuristics
import planner

class MainNode:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/ball_pose', PoseStamped, self.pose_callback)
        self.ball_pose = None

        self.config = Config()

        self.franka_arm = FrankaArm(with_gripper=False)
        self.franka_arm.stop_skill()
        self.franka_arm.reset_joints()

        rospy.Timer(rospy.Duration(0.01), self.update)

        self.state = "waiting for prediction"

        self.state_time = time.time()

    def pose_callback(self, msg):
        self.ball_pose = msg

    def update(self):
        x = [self.ball_pose.pose.position.x, self.ball_pose.pose.position.y, self.ball_pose.pose.position.z]
        v = [self.ball_pose.pose.orientation.x, self.ball_pose.pose.orientation.y, self.ball_pose.pose.orientation.z]

        q = None
        q_pre = None

        if (self.state == "waiting for prediction") and (time.time() - self.state_time < 0.2):
            return

        elif heuristics.time_to_contact(x, v) > self.config.delta_t and self.state == "waiting for prediction":
            self.state = "waiting to swing"
            t, q, q_pre, e_dot_des = planner.get_plan(x, v, self.franka_arm)
            self.franka_arm.goto_joints(q_pre)
        
        elif heuristics.time_to_contact(x, v) < self.config.delta_t:
            self.state = "waiting for prediction"
            self.franka_arm.goto_joints(q + (q - q_pre))

            self.state_time = time.time()
            self.franka_arm.goto_joints(q_pre)
        
    @staticmethod
    def run():
        rospy.spin()


def main():
    main_node = MainNode()
    main_node.run()

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    main()
