import rospy
from geometry_msgs.msg import PoseStamped
from frankapy import FrankaArm


class MainNode:
    def __init__(self):
        rospy.init_node('ball_tracking_ekf', anonymous=True)
        self.subscriber = rospy.Subscriber('/ball_pose', PoseStamped, self.pose_callback)
        self.ball_pose = None

        self.franka_arm = FrankaArm(with_gripper=False)
        self.franka_arm.stop_skill()
        self.franka_arm.reset_joints()

        rospy.Timer(rospy.Duration(0.01), self.update)

    def pose_callback(self, msg):
        self.ball_pose = msg

    def update(self):
        pass

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
