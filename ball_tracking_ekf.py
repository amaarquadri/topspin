import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped


class BallTrackingEKF:
    def __init__(self):
        rospy.init_node('ball_tracking_ekf', anonymous=True)

        rospy.Subscriber('/ball_pose', PoseStamped, self.pose_callback)

        self.x = np.zeros(6)
        self.P = np.eye(6)
        self.last_update_time = rospy.Time.now()
        self.gravity = np.array([0, 0, -9.81])
        self.Q = np.eye(6) * 0.1
        self.R = np.eye(3) * 0.1

    def pose_callback(self, data):
        time = data.header.stamp
        dt = (time - self.last_update_time).to_sec()
        self.last_update_time = time

        # prediction step
        self.x[:3] += self.x[3:] * dt + 0.5 * self.gravity * dt ** 2
        self.x[3:] += self.gravity * dt
        F = np.array([[1, 0, 0, dt, 0, 0],
                      [0, 1, 0, 0, dt, 0],
                      [0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        self.P = F @ self.P + self.P @ F.T + self.Q * dt

        # update step
        z = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        y = z - self.x[:3]
        H = np.eye(3, 6)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ (y - self.x[:3])
