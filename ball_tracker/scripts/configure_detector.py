#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import yaml
import threading
import rospkg

current_frame = None
frame_lock = threading.Lock()

rospack = rospkg.RosPack()

package_path = rospack.get_path('ball_tracker')

def get_console_values():
    value_list = [cv2.getTrackbarPos("Lower Y","Console"),
                  cv2.getTrackbarPos("Lower U","Console"),
                  cv2.getTrackbarPos("Lower V","Console"),
                  cv2.getTrackbarPos("Higher Y","Console"),
                  cv2.getTrackbarPos("Higher U","Console"),
                  cv2.getTrackbarPos("Higher V","Console"),
                  cv2.getTrackbarPos("Erode","Console"),
                  cv2.getTrackbarPos("Dilate","Console"),
                  cv2.getTrackbarPos("Mode","Console"),]
    return value_list


def load_console_values():
    try:
        with open(package_path + "/config/config.yaml","r") as file:
            print("Config Loaded")
            data = yaml.load(file, Loader=yaml.Loader)["param"]
            cv2.setTrackbarPos("Lower Y","Console",data['y'])
            cv2.setTrackbarPos("Lower U","Console",data['u'])
            cv2.setTrackbarPos("Lower V","Console",data['v'])
            cv2.setTrackbarPos("Higher Y","Console",data['Y'])
            cv2.setTrackbarPos("Higher U","Console",data['U'])
            cv2.setTrackbarPos("Higher V","Console",data['V'])
            cv2.setTrackbarPos("Erode","Console",data['erode_iter'])
            cv2.setTrackbarPos("Dilate","Console",data['dil_iter'])
            cv2.setTrackbarPos("Mode","Console",0)
    except Exception as e:
        print(e)
    

def write_console_values():

    dictkeys = ['y','u','v','Y','U','V','erode_iter','dil_iter']
    dictvalues = get_console_values()[:-1]
    config_dict = dict(zip(dictkeys,dictvalues))
    print(config_dict)
    param_dict = {"param" : config_dict}
    with open(package_path + "/config/config.yaml",'w') as file:
        yaml.dump(param_dict,file,default_flow_style=False)
        print("Config Updated")



def image_callback(msg):
    global current_frame
        
    try:
        with frame_lock:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            current_frame = cv_image
    except CvBridgeError as e:
        print(e)

# Initialize ROS node
rospy.init_node('image_processor', anonymous=True)

# Create a CvBridge object
bridge = CvBridge()

# Set up subscription to the ROS image topic
image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)


cv2.namedWindow('Console')
cv2.createTrackbar('Lower Y','Console',0,255,lambda x:x)
cv2.createTrackbar('Lower U','Console',0,255,lambda x:x)
cv2.createTrackbar('Lower V','Console',0,255,lambda x:x)
cv2.createTrackbar('Higher Y','Console',0,255,lambda x:x)
cv2.createTrackbar('Higher U','Console',0,255,lambda x:x)
cv2.createTrackbar('Higher V','Console',0,255,lambda x:x)
cv2.createTrackbar('Erode','Console',0,30,lambda x:x)
cv2.createTrackbar('Dilate','Console',0,30,lambda x:x)
cv2.createTrackbar('Mode','Console',0,2,lambda x:x)




load_console_values()
print("Press 's' to save the configuration")
print("Press 'q' to quit without saving")

while not rospy.is_shutdown():
    with frame_lock:
        if current_frame is not None:
            frame = current_frame.copy()
        else:
            continue

    yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
    y, u, v, Y, U, V, erode_iter, dil_iter, mode = get_console_values()

    mask = cv2.inRange(yuv, np.array([y, u, v]), np.array([Y, U, V]))
    mask = cv2.erode(mask, None, iterations=erode_iter)
    mask = cv2.dilate(mask, None, iterations=dil_iter)
    res = cv2.bitwise_and(current_frame, current_frame, mask=mask)
    if mode == 0:
        cv2.imshow('Console', res)
    elif mode == 1:
        cv2.imshow('Console', mask)
    else:
        cv2.imshow('Console', current_frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    elif cv2.waitKey(1) & 0xFF == ord('s'):
        write_console_values()
        break




cv2.destroyAllWindows()
