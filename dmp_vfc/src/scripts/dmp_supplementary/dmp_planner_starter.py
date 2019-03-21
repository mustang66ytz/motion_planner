#!/usr/bin/env python
import numpy as np
import rospy
from dmp_vfc.msg import *
from std_msgs.msg import Bool
import tf
import tf2_ros
import geometry_msgs.msg
np.set_printoptions(precision=4)

# this script is to allow the interface_module to run correctly
rospy.init_node("no_visual_module", anonymous=True)

marker_pose_eef_pub = rospy.Publisher('marker_position_eef', Waypoint, queue_size=10)
dmp_exe_pub = rospy.Publisher('dmp_execution', Bool, queue_size=10)

endeff_pose_back_msg = Waypoint()
test_goal_left_rot = [-0.6337364484036944, 0.6820241366752285, -0.16401977835833179, 0.3236558038784964]
test_goal_left_trans = [0.1, -0.3, 0.3] # successful, mp6 much better than mp3
endeff_pose_back_msg.target = test_goal_left_rot + test_goal_left_trans

mwc_left_targets_rot = [-0.16423, 0.75264, 0.010383, 0.63754]
mwc_left_targets_trans = [0.16535, -0.28777, 0.27845]

br = tf2_ros.StaticTransformBroadcaster()
dmp_target_transformStamped = geometry_msgs.msg.TransformStamped()
dmp_target_transformStamped.header.stamp = rospy.Time.now()
dmp_target_transformStamped.header.frame_id = '/Back_Y'
dmp_target_transformStamped.child_frame_id = '/DMPTarget'

dmp_target_transformStamped.transform.translation.x = mwc_left_targets_trans[0]
dmp_target_transformStamped.transform.translation.y = mwc_left_targets_trans[1]
dmp_target_transformStamped.transform.translation.z = mwc_left_targets_trans[2]

dmp_target_transformStamped.transform.rotation.x = mwc_left_targets_rot[0]
dmp_target_transformStamped.transform.rotation.y = mwc_left_targets_rot[1]
dmp_target_transformStamped.transform.rotation.z = mwc_left_targets_rot[2]
dmp_target_transformStamped.transform.rotation.w = mwc_left_targets_rot[3]

br.sendTransform(dmp_target_transformStamped)

while not rospy.is_shutdown():
    marker_pose_eef_pub.publish(endeff_pose_back_msg)
    dmp_exe_pub.publish(True)
    rospy.sleep(0.2)

    '''
    br = tf.TransformBroadcaster()

    br.sendTransform(tuple(mwc_left_targets_trans),
                     tuple(mwc_left_targets_rot),
                     rospy.Time.now(),
                     '/DMPTarget',
                     '/Back_Y')
    '''
