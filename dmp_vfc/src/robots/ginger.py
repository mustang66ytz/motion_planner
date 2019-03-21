# !/usr/bin/env python

import roslib

# roslib.load_manifest('visualcontrol')

import rospy
import tf
import Queue
from std_msgs.msg import Float64
import numpy as np
import copy

from sensor_msgs.msg import JointState

from xr1controllerros.msg import ArmMsgs
from xr1controllerros.msg import BodyMsgs
from xr1controllerros.msg import ChainModeChange
from xr1controllerros.msg import HandMsgs
from xr1controllerros.msg import IK_msg
from xr1controllerros.msg import JointAngles

newik_flag = False

if newik_flag:
    from newik.srv import trajectoryIK
# from std_msgs.msg import Float64, Bool


######### global function, supoosed to move to utilis ###########
def euler_2_quaternion(roll, pitch, yaw):
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    quaternion = quaternion.tolist()
    return quaternion


def quaternion_2_euler(quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    return roll, pitch, yaw


def joint_2_armmsg(joint):
    arm_msg = ArmMsgs()
    arm_msg.Shoulder_X = joint[0]
    arm_msg.Shoulder_Y = joint[1]
    arm_msg.Elbow_Z = joint[2]
    arm_msg.Elbow_X = joint[3]
    arm_msg.Wrist_Z = joint[4]
    arm_msg.Wrist_X = joint[5]
    arm_msg.Wrist_Y = joint[6]

    return arm_msg


def armmsg_2_joint(msg):
    joint = [msg.Shoulder_X, msg.Shoulder_Y,
             msg.Elbow_Z, msg.Elbow_X,
             msg.Wrist_Z, msg.Wrist_X, msg.Wrist_Y]
    return joint


def handmsg_2_joint(msg):
    joint = [msg.Thumb, msg.Index, msg.Middle, msg.Ring, msg.Pinky]

    return joint


def joint_2_handmsg(joint):
    hand_msg = HandMsgs()
    hand_msg.Thumb = joint[0]
    hand_msg.Index = joint[1]
    hand_msg.Middle = joint[2]
    hand_msg.Ring = joint[3]
    hand_msg.Pinky = joint[4]

    return hand_msg


def body_2_joint(msg):
    joint = [msg.Knee,
             msg.Back_Z, msg.Back_X, msg.Back_Y,
             msg.Neck_Z, msg.Neck_X,
             msg.Head]
    return joint


def joint_2_body(joint):
    body = BodyMsgs()
    body.Knee = joint[0]
    body.Back_Z = joint[1]
    body.Back_X = joint[2]
    body.Back_Y = joint[3]
    body.Neck_Z = joint[4]
    body.Neck_X = joint[5]
    body.Head = joint[6]

    return body


def float_2_float64(fl):
    f = Float64()
    f.data = fl
    return f


######### Ginger Control Class (Python) ###########
group_select = (
    "LEFT_ARM",
    "RIGHT_ARM",
    "LEFT_HAND",
    "RIGHT_HAND",
    "MAIN_BODY"
)
command_select = (
    "SET_MODE",
    "SET_JOINTS",
    "SET_VEL",
    "SET_CUR"
)


class Ginger(object):

    def __init__(self):

        rospy.init_node('ginger_control_interface')
        self.rospy = rospy
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # the frequency for rate, suppose to under 20 Hz
        self.rate = rospy.Rate(20)

        # self._main_body_joint = Queue.Queue(2)

        # self._left_arm_joint = Queue.Queue(2)
        # self._right_arm_joint = Queue.Queue(2)
        # self._left_arm_vel = Queue.Queue(2)
        # self._right_arm_vel = Queue.Queue(2)
        # self._left_arm_cur = Queue.Queue(2)
        # self._right_arm_cur = Queue.Queue(2)
        #
        # self._left_hand_joint = Queue.Queue(2)
        # self._right_hand_joint = Queue.Queue(2)

        self._main_body_joint = None
        self._left_arm_joint = None
        self._right_arm_joint = None
        self._left_arm_vel = None
        self._right_arm_vel = None
        self._left_arm_cur = None
        self._right_arm_cur = None
        self._left_hand_joint = None
        self._right_hand_joint = None

        ################## Subscriber ############################

        self.rospy.Subscriber("/MainBody/Position", BodyMsgs, self.main_body_joint_CALLBACK)

        self.rospy.Subscriber("/LeftArm/Position", ArmMsgs, self.left_arm_joint_CALLBACK)
        self.rospy.Subscriber("/RightArm/Position", ArmMsgs, self.right_arm_joint_CALLBACK)

        self.rospy.Subscriber("/LeftArm/Velocity", ArmMsgs, self.left_arm_vel_CALLBACK)
        self.rospy.Subscriber("/RightArm/Velocity", ArmMsgs, self.right_arm_vel_CALLBACK)

        self.rospy.Subscriber("/LeftArm/Current", ArmMsgs, self.left_arm_cur_CALLBACK)
        self.rospy.Subscriber("/RightArm/Current", ArmMsgs, self.right_arm_cur_CALLBACK)

        self.rospy.Subscriber("/LeftHand/Position", HandMsgs, self.left_hand_joint_CALLBACK)
        self.rospy.Subscriber("/RightHand/Position", HandMsgs, self.right_hand_joint_CALLBACK)


        #################### Publisher ###########################
        self._main_body_mode_pub = self.rospy.Publisher("/XR1/MainBodyChainModeChange", ChainModeChange, queue_size=5)

        self._left_arm_mode_pub = self.rospy.Publisher("/XR1/LeftArmChainModeChange", ChainModeChange, queue_size=5)
        self._right_arm_mode_pub = self.rospy.Publisher("/XR1/RightArmChainModeChange", ChainModeChange, queue_size=5)

        self._left_hand_mode_pub = self.rospy.Publisher("/XR1/LeftHandChainModeChange", ChainModeChange, queue_size=5)
        self._right_hand_mode_pub = self.rospy.Publisher("/XR1/RightHandChainModeChange", ChainModeChange, queue_size=5)

        ####
        self._main_body_joint_pub = self.rospy.Publisher("/MainBody/TargetPosition", BodyMsgs, queue_size=5)

        self._left_arm_joint_pub = self.rospy.Publisher("/LeftArm/TargetPosition", ArmMsgs, queue_size=5)
        self._right_arm_joint_pub = self.rospy.Publisher("/RightArm/TargetPosition", ArmMsgs, queue_size=5)

        self._left_arm_vel_pub = self.rospy.Publisher("/LeftArm/TargetVelocity", ArmMsgs, queue_size=5)
        self._right_arm_vel_pub = self.rospy.Publisher("/RightArm/TargetVelocity", ArmMsgs, queue_size=5)

        self._left_arm_cur_pub = self.rospy.Publisher("/LeftArm/TargetCurrent", ArmMsgs, queue_size=5)
        self._right_arm_cur_pub = self.rospy.Publisher("/RightArm/TargetCurrent", ArmMsgs, queue_size=5)

        self._left_hand_joint_pub = self.rospy.Publisher("/LeftHand/TargetPosition", HandMsgs, queue_size=5)
        self._right_hand_joint_pub = self.rospy.Publisher("/RightHand/TargetPosition", HandMsgs, queue_size=5)

        self._left_elbow_pub = self.rospy.Publisher("/LeftArm/ElbowAngle", Float64, queue_size=5)
        self._right_elbow_pub = self.rospy.Publisher("/RightArm/ElbowAngle", Float64, queue_size=5)

        self._joint_state_pub = self.rospy.Publisher("/ginger/joint_states", JointState, queue_size=5)

        ### uploading joint state to MMO server
        self.joint_state = JointState()

        self._wheel_name = ["Wheel_left", "Wheel_right", "Wheel_back"]
        self._main_body_name = ["Knee_X", "Back_Z", "Back_X", "Back_Y", "Neck_Z", "Neck_X", "Head_Y"]

        self._left_arm_name = ["Left_Shoulder_X", "Left_Shoulder_Y",
                               "Left_Elbow_Z", "Left_Elbow_X",
                               "Left_Wrist_Z", "Left_Wrist_X", "Left_Wrist_Y"]

        self._right_arm_name = ["Right_Shoulder_X", "Right_Shoulder_Y",
                                "Right_Elbow_Z", "Right_Elbow_X",
                                "Right_Wrist_Z", "Right_Wrist_X", "Right_Wrist_Y"]

        self._left_hand_name = ["Left_1_1", "Left_2_1", "Left_3_1", "Left_4_1", "Left_5_1",
                                "Left_1_2", "Left_2_2", "Left_3_2", "Left_4_2", "Left_5_2",
                                "Left_1_3", "Left_2_3", "Left_3_3", "Left_4_3", "Left_5_3"]
        self._right_hand_name = ["Right_1_1", "Right_2_1", "Right_3_1", "Right_4_1", "Right_5_1",
                                 "Right_1_2", "Right_2_2", "Right_3_2", "Right_4_2", "Right_5_2",
                                 "Right_1_3", "Right_2_3", "Right_3_3", "Right_4_3", "Right_5_3"]

        self.joint_state.name = self._wheel_name + self._main_body_name + self._left_arm_name + self._right_arm_name + \
                                self._left_hand_name + self._right_hand_name

        self.joint_state.position = [0]*54
        self.joint_state.velocity = [0] * 54
        self.joint_state.effort = [0] * 54

        self.rospy.loginfo("Ginger Control")


        # self.derek_ik_server = rospy.ServiceProxy('provide_ik_service', trajectoryIK)

    ####################  Pub jointState  ##################

    def is_all_joint_state_known(self):
        if self._main_body_joint is None:
            rospy.loginfo("_main_body_joint is None, no topic got from /MainBody/Position?")
            return False

        if self._left_arm_joint is None:
            rospy.loginfo("_left_arm_joint is None, no topic got from /LeftArm/Position?")
            return False

        if self._right_arm_joint is None:
            rospy.loginfo("_right_arm_joint is None, no topic got from /RightArm/Position?")
            return False

        if self._left_arm_vel is None:
            rospy.loginfo("_left_arm_vel is None, no topic got from /LeftArm/Velocity?")
            return False

        if self._right_arm_vel is None:
            rospy.loginfo("_right_arm_vel is None, no topic got from /RightArm/Velocity?")
            return False

        if self._left_arm_cur is None:
            rospy.loginfo("_left_arm_cur is None, no topic got from /LeftArm/Current?")
            return False

        if self._right_arm_cur is None:
            rospy.loginfo("_right_arm_cur is None, no topic got from /RightArm/Current?")
            return False

        if self._left_hand_joint is None:
            rospy.loginfo("_left_hand_joint is None, no topic got from /LeftHand/Position?")
            return False

        if self._right_hand_joint is None:
            rospy.loginfo("_right_hand_joint is None, no topic got from /RightHand/Position?")
            return False

        return True

    def pub_joint_state(self):
        if not self.is_all_joint_state_known():
            return

        self.joint_state.position[3:10] = self.get_main_body_joint()
        self.joint_state.position[10:17] = self.get_left_arm_joint()
        self.joint_state.position[17:24] = self.get_right_arm_joint()
        self.joint_state.position[24:29] = self.get_left_hand_joint()
        self.joint_state.position[39:44] = self.get_right_hand_joint()
        self._joint_state_pub.publish(self.joint_state)

    ###### ROS callback function ##############
    def main_body_joint_CALLBACK(self, msg):
        self._main_body_joint = copy.copy(msg)
        # self._main_body_joint.put(msg)

    def left_arm_joint_CALLBACK(self, msg):
        self._left_arm_joint = copy.copy(msg)
    #     self._left_arm_joint.put(msg)
    #
    def right_arm_joint_CALLBACK(self, msg):
        self._right_arm_joint = copy.copy(msg)
    #     self._right_arm_joint.put(msg)
    #
    def left_arm_vel_CALLBACK(self, msg):
        self._left_arm_vel = copy.copy(msg)
    #     self._left_arm_vel.put(msg)
    #
    def right_arm_vel_CALLBACK(self, msg):
        self._right_arm_vel = copy.copy(msg)
    #     self._right_arm_vel.put(msg)
    #
    def left_arm_cur_CALLBACK(self, msg):
        self._left_arm_cur = copy.copy(msg)
    #     self._left_arm_cur.put(msg)
    #
    def right_arm_cur_CALLBACK(self, msg):
        self._right_arm_cur = copy.copy(msg)
    #     self._right_arm_cur.put(msg)
    #
    def left_hand_joint_CALLBACK(self, msg):
        self._left_hand_joint = copy.copy(msg)
    #     self._left_hand_joint.put(msg)
    #
    def right_hand_joint_CALLBACK(self, msg):
        self._right_hand_joint = copy.copy(msg)
    #     self._right_hand_joint.put(msg)

    ################### Set Mode  ##################

    def set_left_arm_mode(self, mode):
        assert type(mode) is int, 'Mode must be 1(position), 2(velocity), 3(current), 4(IK)'
        self.rospy.sleep(0.2)

        self._left_arm_mode_pub.publish(11, mode)

    def set_right_arm_mode(self, mode):
        assert type(mode) is int, 'Mode must be 1(position), 2(velocity), 3(current), 4(IK)'
        self.rospy.sleep(0.2)

        self._right_arm_mode_pub.publish(18, mode)

    def set_left_hand_mode(self, mode):
        assert type(mode) is int, 'Mode must be 1(position), 2(velocity), 3(current), 4(IK)'
        self.rospy.sleep(0.2)

        self._left_hand_mode_pub.publish(25, mode)

    def set_right_hand_mode(self, mode):
        assert type(mode) is int, 'Mode must be 1(position), 2(velocity), 3(current), 4(IK)'
        self.rospy.sleep(0.2)

        self._right_hand_mode_pub.publish(30, mode)

    def set_main_body_mode(self, mode):
        assert type(mode) is int, 'Mode must be 1(position), 2(velocity), 3(current), 4(IK)'
        self.rospy.sleep(0.2)

        self._main_body_mode_pub.publish(4, mode)

    ######Derek IK move linear###################3
    def set_left_arm_EEF_linear(self, trans, quaternion):
        """

        :param trans: <list> [x, y, z]
        :param quaternion: <list> [x, y, z, w]
        :return:
        """

        if newik_flag:
            self.set_left_arm_mode(4)
            while True:
                initial_angles = self.get_left_arm_joint()
                if (initial_angles is not None) and (len(initial_angles) != 0):
                    break

            target = quaternion[:4] + trans

            current_pos = self.get_left_arm_EEF()

            # list = []
            # for i, value in enumerate(target):
            #     list.append(value)
            # self.derek_ik_pub = rospy.Publisher('trajectory_pose', target, queue_size = 10)
            plan_mode = 2
            hand_selection = 0
            num_points = 10
            rospy.wait_for_service('provide_ik_service')
            resp = None
            try:
                derek_ik_server = rospy.ServiceProxy('provide_ik_service', trajectoryIK)
                # resp = derek_ik_server(plan_mode, initial_angles, target)

                resp = derek_ik_server(hand_selection, plan_mode, num_points, initial_angles, target) # 0: left_hand; plan_mode = 2, number of points returnes 10, initial_angles, target
                if resp.success == 0:
                    print "init angle:", initial_angles
            except rospy.ServiceException, e:
                print(e)

            # resp = derek_ik_server(plan_mode, initial_angles, target)

            if resp is not None and resp.success :

                target_joint_angles = [i for i in resp.jointAngles]
                num_joints = len(initial_angles)
                num_points = len(target_joint_angles)/num_joints
                target_joint_trajectory = np.reshape(target_joint_angles, (num_points, num_joints))
                for i in range(num_points):
                    waypoint = target_joint_trajectory[:][i].tolist()
                    if np.all([a != 0 for a in waypoint]):
                        self.set_left_arm_joint(waypoint, interpolation=1)
                    else:
                        print "hit a zero point, index:", i
                        return False, resp
                print ("success, but with failed points: d%", len(resp.failNum)/float(num_points))
                # duration = 0.001*len(resp.jointAngles)
                # print(duration)
                # rospy.sleep(duration)
                return True, resp
            else:
                print("ik failed")
                return False, resp
        else:
            self.set_left_arm_EEF(trans, quaternion)



    #################### Main Body  ##################

    def get_main_body_joint(self):
        """

        :return: Get main body joint
        """
        joint = body_2_joint(self._main_body_joint)

        if joint is None:
            raise ValueError('No main body joint angles. Please check the system setup')
        return joint

    def set_main_body_joint(self, target_joint, interpolation=None, accuracy=0.008, send_times=1):
        """

        :param target_joint: Set main body target.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
        :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
        :return:
        """
        assert type(target_joint) is list, 'Joint angle input is list'

        if interpolation is not None:
            assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'
            for i in range(send_times):
                self._main_body_joint_pub.publish(joint_2_body(target_joint))


        if not self.rospy.is_shutdown():

            if interpolation is None or interpolation == 0:
                self._main_body_joint_pub.publish(joint_2_body(target_joint))
            else:
                cur_joint = self.get_main_body_joint()
                unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]

                for index in range(1, interpolation + 1):
                    tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
                    while not self.rospy.is_shutdown():
                        error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, self.get_main_body_joint())]

                        if index == interpolation and np.all([e < accuracy for e in error_joint]):
                            print('Target arrived')
                            break
                        elif np.all([e < 10 * accuracy for e in error_joint]):
                            break
                        self._main_body_joint_pub.publish(joint_2_body(tmp_target))
                        rospy.sleep(0.05)


    def get_head_EEF(self):
        """

        :return: Head transformation with respect ot Back_Y frame
        """
        self.listener.waitForTransform('/Back_Y', '/Head', self.rospy.Time(), self.rospy.Duration(0.1))
        pose = self.listener.lookupTransform('/Back_Y', '/Head', self.rospy.Time(0))
        trans = pose[0]
        qua = pose[1]

        return trans, qua

    def set_head_movement(self, neck_z=None, neck_x=None):
        """

        :param neck_z: Head horizontally rotation. Right(>0) - Left(<0)
        :param neck_x: Head vertically rotation. Up(>0) - Down(<0)
        :return:
        """
        # assert (type(neck_z) is float or type(neck_z) is int ) and (
        # type(neck_x) is float or type(neck_z) is int), 'Head joint is float number, please check'
        assert -1 <= neck_x <= 0.5 or neck_x is None, 'Neck_x setting is out of permitted range, please check set_head_movement()'
        assert -0.87 <= neck_z <= 0.87or neck_z is None, 'Neck_z setting is out of permitted range, please check set_head_movement()'

        if not self.rospy.is_shutdown():
            self.rospy.sleep(.5)
            joint = self.get_main_body_joint()
            joint = copy.copy(joint)
            if neck_z is not None:
                joint[-3] = neck_z
            if neck_x is not None:
                joint[-2] = neck_x
            self.rospy.sleep(0.2)
            self._main_body_joint_pub.publish(joint_2_body(joint))

    def set_main_body_movement(self, back_z=None, back_x=None):
        """

        :param back_z: Back side rotation. Right(>0) - Left(<0)
        :param back_x: Back rotation. Forward(>0)
        :return:
        """
        assert -1.6 <= back_z <= 1.6 or back_z is None, 'Back_z is out of permitted range, please check set_main_body_movement()'
        assert 0 <= back_x <= 0.8 or back_x is None, 'Back_x is out of permitted range, please check set_main_body_movement()'

        if not self.rospy.is_shutdown():
            joint = self.get_main_body_joint()
            joint[0] = 0
            if back_z is not None:
                joint[1] = back_z
            if back_x is not None:
                joint[2] = back_x
            self.rospy.sleep(0.2)
            self._main_body_joint_pub.publish(joint_2_body(joint))

    #################### Left Arm  ##################
    ### get-set pairs ######

    def get_left_arm_joint(self):
        """

        :return: Get left arm joint
        """
        # joint = armmsg_2_joint(self._left_arm_joint.get())
        joint = armmsg_2_joint(self._left_arm_joint)

        if joint is None:
            raise ValueError('No left arm joint angles. Please check the system setup')
        return joint

    def set_left_arm_joint(self, target_joint, interpolation=None,
                           accuracy=0.008, send_times=1):
        """

        :param target_joint: Set left arm target.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
        :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
        :return:
        """

        assert type(target_joint) is list, 'Joint angle input is list'

        if interpolation is not None:
            assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'

        if not self.rospy.is_shutdown():

            if interpolation is None or interpolation == 0:
                for i in range(send_times):
                    self._left_arm_joint_pub.publish(joint_2_armmsg(target_joint))

            else:
                cur_joint = self.get_left_arm_joint()
                unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]

                for index in range(1, interpolation + 1):
                    tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
                    sleep_cnt = 0
                    while not self.rospy.is_shutdown():
                        error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, self.get_left_arm_joint())]

                        if index == interpolation and np.all([e < accuracy for e in error_joint]):
                            print('Target arrived')
                            break
                        elif np.all([e < 5 * accuracy for e in error_joint]):
                            break
                        sleep_cnt += 1
                        if (sleep_cnt % 10) == 4:
                            rospy.loginfo("error_joint: %s" % error_joint)
                            self._left_arm_joint_pub.publish(joint_2_armmsg(tmp_target))
                        rospy.sleep(0.05)

    def get_left_arm_velocity(self):
        """

        :return: Get left arm velocity
        """
        # velocity = armmsg_2_joint(self._left_arm_vel.get())
        velocity = armmsg_2_joint(self._left_arm_vel)

        if velocity is None:
            raise ValueError('No left arm velocity. Please check the system setup')
        return velocity

    def set_left_arm_velocity(self, target_velocity, interpolation=None, accuracy=0.01):
        """

        :param target_velocity: Set left arm velocity.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target velocity.
        :param accuracy: Target velocity accuracy, 0.008 is roughly about 1 degree/sec.
        :return:
        """
        assert type(target_velocity) is list, 'Velocity input is list'

        while not self.rospy.is_shutdown():
            self._left_arm_vel_pub.publish(joint_2_armmsg(target_velocity))
            error_vel = [np.abs(s - g) for (s, g) in zip(target_velocity, self.get_left_arm_velocity())]
            print(error_vel)
            if np.all([e < accuracy for e in error_vel]):
                print('Target velocity achieved')
                break

    def get_left_arm_current(self):
        """

        :return: Get left arm current
        """
        # current = armmsg_2_joint(self._left_arm_cur.get())
        current = armmsg_2_joint(self._left_arm_cur)

        if current is None:
            raise ValueError('No left arm velocity. Please check the system setup')
        return current

    def set_left_arm_current(self, target_current, interpolation=None, accuracy=0.01):
        """

        :param target_current: Set left arm current.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target current.
        :param accuracy: Target current accuracy, 0.008 is roughly about 1 degree/sec^2.
        :return:
        """
        assert type(target_current) is list, 'Velocity input is list'

        while not self.rospy.is_shutdown():
            self._left_arm_cur_pub.publish(joint_2_armmsg(target_current))
            error_cur = [np.abs(s - g) for (s, g) in zip(target_current, self.get_left_arm_current())]
            print(error_cur)
            if np.all([e < accuracy for e in error_cur]):
                print('Target current achieved')
                break

    def get_left_arm_EEF(self):
        """

        :return: Get left arm End-Effector pose in translation, quaternion
        """
        self.listener.waitForTransform('/Back_Y', '/LeftEndEffector', self.rospy.Time(), self.rospy.Duration(0.1))
        pose = self.listener.lookupTransform('/Back_Y', '/LeftEndEffector', self.rospy.Time(0))
        trans = pose[0]
        qua = pose[1]

        return trans, qua

    def set_left_arm_EEF(self, trans, quaternion):
        """

        :param trans: <list> [x, y, z]
        :param quaternion: <list> [x, y, z, w]
        :return:
        """
        # translation = tuple([t * -1 for t in trans])
        self.broadcaster.sendTransform(trans,
                                       tuple(quaternion),
                                       rospy.Time.now(),
                                       '/LeftEndEffectorTarget',
                                       '/Back_Y')

    #################### Right Arm ##################

    def get_right_arm_joint(self):
        """

        :return: Get right arm joint
        """
        # joint = armmsg_2_joint(self._right_arm_joint.get())
        joint = armmsg_2_joint(self._right_arm_joint)

        if joint is None:
            raise ValueError('No right arm joint angles. Please check the system setup')
        return joint

    def set_right_arm_joint(self, target_joint, interpolation=None,
                            accuracy=0.008, send_times=10):
        """

        :param target_joint: Set right arm target.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
        :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
        :return:
        """
        assert type(target_joint) is list, 'Joint angle input is list'

        if interpolation is not None:
            assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'

        if not self.rospy.is_shutdown():

            if interpolation is None or interpolation == 0:
                for i in range(send_times):
                    self._right_arm_joint_pub.publish(joint_2_armmsg(target_joint))

            else:
                cur_joint = self.get_right_arm_joint()
                unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]

                for index in range(1, interpolation + 1):
                    tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
                    while not self.rospy.is_shutdown():
                        error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, self.get_right_arm_joint())]

                        if index == interpolation and np.all([e < accuracy for e in error_joint]):
                            # print('Target arrived')
                            break
                        elif np.all([e < 5 * accuracy for e in error_joint]):
                            break
                        self._right_arm_joint_pub.publish(joint_2_armmsg(tmp_target))



    def get_right_arm_velocity(self):
        """

        :return: Get right arm velocity
        """
        velocity = armmsg_2_joint(self._right_arm_vel)
        # velocity = armmsg_2_joint(self._right_arm_vel.get())

        if velocity is None:
            raise ValueError('No right arm velocity. Please check the system setup')
        return velocity

    def set_right_arm_velocity(self, target_velocity, interpolation=None, accuracy=0.01):
        """

        :param target_velocity: Set left arm velocity.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target velocity.
        :param accuracy: Target velocity accuracy, 0.008 is roughly about 1 degree/sec.
        :return:
        """
        assert type(target_velocity) is list, 'Velocity input is list'

        while not self.rospy.is_shutdown():
            self._right_arm_vel_pub.publish(joint_2_armmsg(target_velocity))
            error_vel = [np.abs(s - g) for (s, g) in zip(target_velocity, self.get_right_arm_velocity())]
            if np.all([e < accuracy for e in error_vel]):
                print('Target velocity achieved')
                break

    def get_right_arm_current(self):
        """

        :return: Get right arm current
        """
        # current = armmsg_2_joint(self._right_arm_cur.get())
        current = armmsg_2_joint(self._right_arm_cur)

        if current is None:
            raise ValueError('No left arm velocity. Please check the system setup')
        return current

    def set_right_arm_current(self, target_current, interpolation=None, accuracy=0.01):
        """

        :param target_current: Set left arm current.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target current.
        :param accuracy: Target current accuracy, 0.008 is roughly about 1 degree/sec^2.
        :return:
        """
        assert type(target_current) is list, 'Velocity input is list'

        while not self.rospy.is_shutdown():
            self._right_arm_cur_pub.publish(joint_2_armmsg(target_current))
            error_cur = [np.abs(s - g) for (s, g) in zip(target_current, self.get_right_arm_current())]
            if np.all([e < accuracy for e in error_cur]):
                print('Target current achieved')
                break

    def get_right_arm_EEF(self):
        """

        :return: Get right arm End-Effector pose in translation, quaternion
        """
        self.listener.waitForTransform('/Back_Y', '/RightEndEffector', self.rospy.Time(), self.rospy.Duration(0.1))
        pose = self.listener.lookupTransform('/Back_Y', '/RightEndEffector', self.rospy.Time(0))
        trans = pose[0]
        qua = pose[1]

        return trans, qua

    def set_right_arm_EEF(self, trans, quaternion):
        """

        :param trans: <list> [x, y, z]
        :param quaternion: <list> [x, y, z, w]
        :return:
        """
        # translation = tuple([t * -1 for t in trans])
        # trans = [trans[0] - 0.04, trans[1] + 0.05, trans[2]]
        self.broadcaster.sendTransform(trans,
                                       tuple(quaternion),
                                       rospy.Time.now(),
                                       '/RightEndEffectorTarget',
                                       '/Back_Y')

    def set_left_elbow(self, angle):
        """

        :param angle: Set left arm elbow angle for IK mode(4)
        :return:
        """
        assert type(angle) is float, 'ElbowAngle input is float'
        self._left_elbow_pub.publish(float_2_float64(angle))

    def set_right_elbow(self, angle):
        """

        :param angle: Set right arm elbow angle for IK mode(4)
        :return:
        """
        assert type(angle) is float, 'ElbowAngle input is float'
        self._right_elbow_pub.publish(float_2_float64(angle))

    ################### Hand #########################

    def get_left_hand_joint(self):
        """

        :return: Get left hand joint
        """
        # joint = handmsg_2_joint(self._left_hand_joint.get())
        joint = handmsg_2_joint(self._left_hand_joint)

        if joint is None:
            raise ValueError('No left hand joint. Please check the system setup')
        return joint

    def set_left_hand_joint(self, joint):
        """

        :return: Set left hand joint
        """
        assert type(joint) is list, 'Joint input is list'
        self.rospy.sleep(.1)
        self._left_hand_joint_pub.publish(joint_2_handmsg(joint))

    def get_right_hand_joint(self):
        """

        :return: Get left hand joint
        """
        # joint = handmsg_2_joint(self._left_hand_joint.get())
        joint = handmsg_2_joint(self._right_hand_joint)

        if joint is None:
            raise ValueError('No right hand joint. Please check the system setup')
        return joint

    def set_right_hand_joint(self, joint):
        """

        :return: Set left hand joint
        """

        assert type(joint) is list, 'Joint input is list'
        self.rospy.sleep(.1)
        self._right_hand_joint_pub.publish(joint_2_handmsg(joint))

    def close_gripper(self, arm, degree=8):
        """

        :param arm: which arm to close gripper
        :param degree: close gripper degree
        :return:
        """
        assert arm == 'left' or arm == 'right'
        assert 0<=degree<=20, 'Grasping level can only be within 0-10'
        self.rospy.sleep(.3)

        d = degree/10.0
        if arm == 'left':
            self.set_left_hand_joint([d, d, d, d, d])
        elif arm== 'right':
            self.set_right_hand_joint([d]*5)


    def open_gripper(self, arm ,degree = 0.0):

        assert arm == 'left' or arm == 'right', 'Please input the right hand'
        assert  -5<=degree<=0, 'degree must within -5 to 0'
        self.rospy.sleep(.1)

        d = degree / 10.0

        if arm == 'left':
            self.set_left_hand_joint([d]*5)
        elif arm== 'right':
            self.set_right_hand_joint([d]*5)


    ########## utilis functions ###########################
    def go_zero(self):
        self.set_main_body_mode(1)
        self.set_left_hand_mode(1)
        self.set_right_hand_mode(1)
        self.set_left_arm_mode(1)
        self.set_right_arm_mode(1)

        self.open_gripper('left')
        self.open_gripper('right')
        self.rospy.sleep(.5)


        self.set_main_body_joint([0, 0, 0, 0, 0, 0, 0], interpolation=None, send_times=10)
        self.rospy.sleep(.5)

        self.set_left_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=None, send_times=10)
        self.set_right_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=None, send_times=10)


    def go_sleep(self):
        self.set_main_body_mode(1)
        self.set_left_hand_mode(1)
        self.set_right_hand_mode(1)
        self.set_left_arm_mode(1)
        self.set_right_arm_mode(1)

        self.set_left_hand_joint([0, 0, 0, 0, 0])
        self.set_right_hand_joint([0, 0, 0, 0, 0])
        self.rospy.sleep(1.)

        self.set_left_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
        self.set_right_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
        self.rospy.sleep(1.)


        self.set_main_body_joint([-0.678099835449, 0.116697004962,
                                  1.01603631491, 0.045180407687,
                                  0.0979084543285, 0.693636284968, 0.0], interpolation=100, accuracy=0.1)

        self.rospy.sleep(1.)

        self.set_left_arm_joint([-0.3, 0, 0, 0, 0, 0, 0], interpolation=1)
        self.set_right_arm_joint([-0.3, 0, 0, 0, 0, 0, 0], interpolation=1)
