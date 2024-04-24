#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import yaml
import rospkg

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# K: [425.19189453125, 0.0, 422.978515625, 0.0, 424.6562805175781, 242.1155242919922, 0.0, 0.0, 1.0]


class EKF:
    def __init__(self):
        self.x = np.zeros(6) # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 0.1 # Covariance matrix
        self.Q = np.eye(6) * 0.1 # Process noise
        self.R = np.eye(3) * 0.1 # Measurement noise
        self.gravity = np.array([0, 0, -9.81])


        self.last_time = rospy.Time.now()

    def predict(self,dt = None):
        if dt is None:
            time = rospy.Time.now()
            dt = (time - self.last_time).to_sec()
        # print(f"dt: {dt}")
        # self.last_time = time

        self.x[:3] += self.x[3:] * dt + 0.5 * self.gravity * dt ** 2
        self.x[3:] += self.gravity * dt
        F = np.array([[1, 0, 0, dt, 0, 0],
                      [0, 1, 0, 0, dt, 0],
                      [0, 0, 1, 0, 0, dt],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, 0, 0, 0, 1]])
        self.P = F @ self.P + self.P @ F.T + self.Q * dt

        return self.x[:3]

    def update(self, z):
        self.last_time = rospy.Time.now()
        y = z - self.x[:3]
        H = np.eye(3, 6)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S) # Kalman gain

        # Update state estimate
        self.x += K @ y

        # Update covariance matrix
        self.P = (np.eye(6) - K @ H) @ self.P

        return self.x[:3]
    
    def get_state(self):
        return self.x



class BallTracker:
    def __init__(self):

        self.DEBUG = True
        self.PX, self.PY = None, None

        
        # Camera Intrinsics
        self.fx =  425.19189453125
        self.fy =  424.6562805175781
        self.cx =  422.978515625
        self.cy =  242.1155242919922
        
        # ROS Package Path
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ball_tracker')
        
        # ROS Subscribers
        self.bridge = CvBridge()
        rospy.init_node('ball_tracker', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)

        # ROS Publishers
        self.marker_pub = rospy.Publisher('/ball_marker', Marker, queue_size=10)
        self.markers_pub = rospy.Publisher('/ball_markers', MarkerArray, queue_size=10)
        self.path_pub1 = rospy.Publisher('/ball_path', Path, queue_size=10)
        self.path_pub2 = rospy.Publisher('/ekf_path', Path, queue_size=10)
        self.pose_pub = rospy.Publisher('/ball_pose', PoseStamped, queue_size=10)

        self.ekf = EKF()
        self.ball_poses = []
        self.ekf_poses = []

        print("Ball tracker is running...")

    def load_values(self):
        with open(f"{self.package_path}/config/config.yaml", "r") as file:
            data = yaml.safe_load(file)["param"]
            values = [data['y'], data['u'], data['v'], data['Y'], data['U'],
                      data['V'], data['erode_iter'], data['dil_iter']]
            return values
        
    def publish_marker(self, x, y, z, frame_id="camera_color_optical_frame"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ball"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 0.91
        marker.color.g = 0.59
        marker.color.b = 0.20
        self.marker_pub.publish(marker)

    def publish_markers(self, poses):
        marker_array = MarkerArray()

        for i, pose in enumerate(poses):
            marker = Marker()
            marker.header.frame_id = "camera_color_optical_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "markers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]  # raw xyz
            marker.pose.position.y = pose[1]
            marker.pose.position.z = pose[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.04
            marker.scale.y = 0.04
            marker.scale.z = 0.04

            # Assigning different colors to each marker
            if i < len(poses) / 3:  # raw xyz markers
                marker.color.r = 0.91
                marker.color.g = 0.59
                marker.color.b = 0.20
            elif i < 2 * len(poses) / 3:  # ekf xyz markers
                marker.color.r = 0.20
                marker.color.g = 0.91
                marker.color.b = 0.59
            else:  # pred xyz markers
                marker.color.r = 0.59
                marker.color.g = 0.20
                marker.color.b = 0.91

            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.markers_pub.publish(marker_array)

    def publish_path(self, mode="raw"):
        path = Path()
        path.header.frame_id = "camera_color_optical_frame"
        path.header.stamp = rospy.Time.now()

        if mode == "raw":
            poses = self.ball_poses
            publisher = self.path_pub1
        elif mode == "ekf":
            poses = self.ekf_poses
            publisher = self.path_pub2

        if len(poses) > 20:
            poses = poses[-20:]

        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = pose[2]
            path.poses.append(pose_stamped)

        publisher.publish(path)


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            yuv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YUV)
            y, u, v, Y, U, V, erode_iter, dil_iter = self.load_values()

            lower = np.array([y, u, v])
            upper = np.array([Y, U, V])
            mask = cv2.inRange(yuv, lower, upper)
            mask = cv2.erode(mask, None, iterations=erode_iter)
            mask = cv2.dilate(mask, None, iterations=dil_iter)
            res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)

                if radius > 0.5:
                    cv2.circle(res, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.putText(res, "Ball", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(res,"Ball Detected",(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                    self.PX, self.PY = int(x), int(y)
                else:
                    print("Radius too small. Ignoring...")
                    self.PX, self.PY = None, None
            else:
                cv2.putText(res, "No Ball Detected", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                self.PX, self.PY = None, None

            if self.DEBUG:
                cv2.imshow("Tracker", res)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Shutting down...")
                    cv2.destroyAllWindows()
                    rospy.signal_shutdown("User exited")

        except CvBridgeError as e:
            print(e)

    def depth_callback(self, msg):
        if self.PX is not None and self.PY is not None:
            try:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                depth = depth_image[self.PY, self.PX]
                # print(f"Depth at ({self.PX}, {self.PY}): {depth} mm")
                Z = depth
                X = (self.PX - self.cx) * Z / self.fx
                Y = (self.PY - self.cy) * Z / self.fy

                # Convert to meters
                x, y, z = X / 1000, Y / 1000, Z / 1000
                # print(f"Coordinates: ({X}, {Y}, {Z})")
                
                poses = []
                poses.append([x, y, z])
                self.ball_poses.append([x, y, z])
                # self.publish_marker(x, y, z)
                # print(f"Raw Coordinates: ({x}, {y}, {z})")

                # EKF
                x, y, z = self.ekf.update(np.array([x, y, z]))
                x, y, z = self.ekf.predict(0.001)
                poses.append([x, y, z])
                self.ekf_poses.append([x, y, z])
                # print(f"EKF Coordinates: ({x}, {y}, {z})")

                pose = PoseStamped()
                pose.header.frame_id = "camera_color_optical_frame"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                self.pose_pub.publish(pose)



                # Prediction
                vx, vy, vz = self.ekf.get_state()[3:]
                dT = 0.001
                x += vx * dT
                y += vy * dT
                z += vz * dT + 0.5 * -9.81 * dT ** 2
                
                poses.append([x, y, z])



                # self.publish_marker(x, y, z)
                self.publish_markers(poses)
                self.publish_path("raw")
                self.publish_path("ekf")
            except CvBridgeError as e:
                print(e)

    def run(self):
        rospy.spin()

        if self.DEBUG:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = BallTracker()
    tracker.run()
