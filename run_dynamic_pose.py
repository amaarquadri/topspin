import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight
import matplotlib.pyplot as plt

"""
Home pose for racket
Tra: [0.42507898 0.08112724 0.19879967]
 Rot: [[ 3.10849424e-03 -9.95751192e-01  9.19282933e-02]
 [ 2.86354611e-02  9.19787913e-02  9.95349051e-01]
 [-9.99575457e-01 -4.61627724e-04  2.88002645e-02]]
 Qtn: [ 0.53006781 -0.46966192  0.51479439  0.48313943]
 from franka_tool to world
Joints: [-1.71721697  0.93825313  1.16560006 -2.42582475  0.63423065  1.98583073
  1.53964265]

"""

import rospy
import numpy as np


if __name__ == "__main__":
    fa = FrankaArm(with_gripper=False)
    # fa.reset_joints()

    rospy.loginfo('Generating Trajectory')
    p0 = fa.get_pose()
    # p1 = p0.copy()
    # T_delta = RigidTransform(
    #     translation=np.array([0.0, 0, 0.0]),
    #     rotation=RigidTransform.y_axis_rotation(np.deg2rad(10)), 
    #                         from_frame=p1.from_frame, to_frame=p1.from_frame)
    j = [-1.71721697,  0.93825313,  1.16560006, -2.42582475,  0.63423065,  1.98583073, 1.53964265]
    fa.goto_joints(j)
    p1 = fa.get_pose()

    T = 50
    dt = 0.02
    ts = np.arange(0, T, dt)

    A = 0.10
    w = 4
    weights = [min_jerk_weight(t, T) for t in ts]
    # pose_traj = [p1.interpolate_with(p0, w) for w in weights]
    offsets = list(A * np.sin(w * ts))
    pose_traj = [ RigidTransform(
                translation=np.array([p1.translation[0], p1.translation[1], p1.translation[2]+offset]),
                rotation=p1.rotation, 
                from_frame='franka_tool', to_frame='world') for offset in offsets]
    # pose_traj = [p1 * offset for offset in offsets]
    # plt.plot([pose.translation[2] for pose in pose_traj])
    # plt.show()
    # for pose in pose_traj:
    #     print(pose.translation)
    # exit()

    z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]
    imp = 2 # impedance scaling

    rospy.loginfo('Initializing Sensor Publisher')
    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    rate = rospy.Rate(1 / dt)

    rospy.loginfo('Publishing pose trajectory...')

    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=10,
        cartesian_impedances=FC.DEFAULT_CARTESIAN_IMPEDANCES
    )
    init_time = rospy.Time.now().to_time()

    poses = []
    for i in range(2, len(ts)):
        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, timestamp=timestamp, 
            position=pose_traj[i].translation, quaternion=pose_traj[i].quaternion
        )
        fb_ctrlr_proto = CartesianImpedanceSensorMessage(
            id=i, timestamp=timestamp,
            translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES[:2] + [z_stiffness_traj[i]],
            rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
            )

        poses.append(fa.get_pose())
        rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
        pub.publish(ros_msg)
        rate.sleep()

    np.save(f'impedance_{imp}', np.array(poses))
    np.save('reference_traj', np.array(pose_traj))
    
    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    pub.publish(ros_msg)

    rospy.loginfo('Done')