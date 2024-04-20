import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm
from config import Config
from heuristics import Heuristics
from planner import Planner

class MainNode:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/ball_pose', PoseStamped, self.pose_callback)
        self.ball_pose = None

        self.config = Config()
        self.heuristics = Heuristics()
        self.planner = Planner()

        self.franka_arm = FrankaArm(with_gripper=False)
        self.franka_arm.stop_skill()
        self.franka_arm.reset_joints()

        rospy.Timer(rospy.Duration(0.01), self.update)

        self.state = "waiting for prediction"

    def pose_callback(self, msg):
        self.ball_pose = msg

    def update(self):

        if self.ball_pose.pose.position.z - self.config.z_paddle < self.config.delta_z:
            self.state = "waiting to swing"

        x = self.ball_pose.pose.position.x
        y = self.ball_pose.pose.position.y
        z = self.ball_pose.pose.position.z

        # @Amaar TODO
        vx = self.ball_pose.pose.orientation.vx
        vy = self.ball_pose.pose.orientation.vy
        vz = self.ball_pose.pose.orientation.vz

        t, q, q_pre = self.planner.get_plan(x, y, z, vx, vy, vz)
        isComplete = self.planner.execute_plan(t, q, q_pre)
        

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
