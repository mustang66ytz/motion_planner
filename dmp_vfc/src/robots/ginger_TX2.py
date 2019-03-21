# !/usr/bin/env python

import roslib

# roslib.load_manifest('visualcontrol')

import rospy
import tf
import Queue
from std_msgs.msg import Float64
import numpy as np

from sensor_msgs.msg import JointState

from copy import deepcopy


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


class GingerTX2(object):

    def __init__(self):

        rospy.init_node('ginger_TX2_control_interface')
        self.rospy = rospy
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        # self.joint_state = None
        self.joint_state = JointState()


        self._main_body_name = ["Back_Z", "Back_X", "Back_Y", "Neck_Z", "Neck_X", "Head_Y"]

        # self._left_arm_name = ["Left_Shoulder_X", "Left_Shoulder_Y",
        #                        "Left_Elbow_Z", "Left_Elbow_X",
        #                        "Left_Wrist_Z", "Left_Wrist_Y", "Left_Wrist_X"]
        #
        # self._right_arm_name = ["Right_Shoulder_X", "Right_Shoulder_Y",
        #                         "Right_Elbow_Z", "Right_Elbow_X",
        #                         "Right_Wrist_Z", "Right_Wrist_Y", "Right_Wrist_X"]

        # TODO: check the order of Wrist_X and Wrist_Y
        self._left_arm_name = ["Left_Shoulder_X", "Left_Shoulder_Y",
                               "Left_Elbow_Z", "Left_Elbow_X",
                               "Left_Wrist_Z", "Left_Wrist_X", "Left_Wrist_Y"]

        self._right_arm_name = ["Right_Shoulder_X", "Right_Shoulder_Y",
                                "Right_Elbow_Z", "Right_Elbow_X",
                                "Right_Wrist_Z", "Right_Wrist_X", "Right_Wrist_Y"]

        self._left_hand_name = ["Left_1_1", "Left_2_1", "Left_3_1", "Left_4_1", "Left_5_1"]
        self._right_hand_name = ["Right_1_1", "Right_2_1", "Right_3_1", "Right_4_1", "Right_5_1"]

        # the frequency for rate, suppose to under 20 Hz
        self.rate = rospy.Rate(20)

        self.rospy.Subscriber("/ginger/joint_states", JointState, self._CALLBACK)

        self._ginger_pub = self.rospy.Publisher("/ginger/vfc_motion", JointState, queue_size=5)

        self.rospy.loginfo("Ginger Control")

    def _CALLBACK(self, msg):
        # should only assign when the data is not None
        if msg is not None:
            self.joint_state = msg

#################_get_joint_stats##################
    def _get_joint_states(self):

        zero_time = rospy.Time().now()
        while True:
            now = rospy.Time.now()
            joint = list(self.joint_state.position)
            if (len(joint) != 0) and (joint is not None):
                break
            if now - zero_time > rospy.Duration(0.3):
                raise ValueError('get right arm position failed, please check the system')

        if joint is None:
            raise ValueError('No right arm joint angles. Please check the system setup')
        return joint

    def get_main_body_joint(self):
        return self._get_joint_states()[:6]

    def get_left_arm_joint(self):
        return self._get_joint_states()[6:13]

    def get_right_arm_joint(self):
        return self._get_joint_states()[13:20]


    #################### Main Body  ##################
    def _set_robot_group_joint(self, robot_group_name, target_joint, interpolation=None, accuracy=0.008):


        """

        :param target_joint: Set main body target.
        :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
        :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
        :return:
        """
        assert type(target_joint) is list, 'Joint angle input is list'

        if interpolation is not None:
            assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'

        robot_group = JointState()
        if robot_group_name == "main_body":
            robot_group.name = self._main_body_name
        elif robot_group_name == "left_arm":
            robot_group.name = self._left_arm_name
        elif robot_group_name == "right_arm":
            robot_group.name = self._right_arm_name

        assert len(target_joint) == len(robot_group.name), "target_joint and robot_group.name size mismatch"
        size = len(robot_group.name)

        if not self.rospy.is_shutdown():

            if interpolation is None or interpolation == 0:
                robot_group.position = target_joint
                self._ginger_pub.publish(robot_group)
            else:
                cur_joint = []
                while True:
                    if len(cur_joint) != size:
                        # print("get_joint list is empty")
                        if robot_group_name == "main_body":
                            cur_joint= self.get_main_body_joint()
                        elif robot_group_name == "left_arm":
                            cur_joint= self.get_left_arm_joint()
                        elif robot_group_name == "right_arm":
                            cur_joint= self.get_right_arm_joint()
                        else:
                            raise ValueError('wrong robot_group_name')
                    else:
                        break
                if len(cur_joint) != size:
                    print("get_joint failed")
                    return False

                unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]

                for index in range(1, interpolation + 1):
                    tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
                    robot_group.position = tmp_target


                    now = rospy.Time().now()
                    zero_time = rospy.Time().now()
                    count = 0
                    count_flag = False

                    pre_error_joint = cur_joint
                    pre_error_joint = [np.inf for i in pre_error_joint]

                    while not self.rospy.is_shutdown():
                        if robot_group_name == "main_body":
                            cur_joint= self.get_main_body_joint()
                        elif robot_group_name == "left_arm":
                            cur_joint= self.get_left_arm_joint()
                        elif robot_group_name == "right_arm":
                            cur_joint= self.get_right_arm_joint()
                        else:
                            raise ValueError('wrong robot_group_name')

                        error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, cur_joint)]



                        if index == interpolation and np.all([e < accuracy for e in error_joint]):
                            print('Target arrived')
                            break
                        elif np.all([e < 5 * accuracy for e in error_joint]):
                            break

                        # check error change, if lower than 0.0000001, then add counter
                        diff_error_sum = np.sum([np.abs(a-b) for (a,b) in zip(pre_error_joint, error_joint)])
                        if diff_error_sum < 0.001:
                            if count_flag is False:
                                zero_time = rospy.Time().now()
                                now = rospy.Time().now()
                                count_flag = True
                                count = 0
                            else:
                                now = rospy.Time().now()
                                count += 1

                        else:
                            zero_time = rospy.Time().now()
                            now = rospy.Time().now()
                            count_flag = False
                            count = 0


                        if now - zero_time > rospy.Duration(1):
                            print("accuracy not reached, time reached, but move on")
                            break

                        if count > 20000:
                            print(count)
                            print("accuracy not reached, counter reached, but move on")
                            break

                        # time out
                        # now = rospy.Time().now()
                        # if now - zero_time > rospy.Duration(0.3):
                        #     raise ValueError('get right arm position failed, please check the system')

                        self._ginger_pub.publish(robot_group)
                        pre_error_joint = error_joint
        return True

    def set_main_body_joint(self, target_joint, interpolation=None, accuracy=0.008):
        return self._set_robot_group_joint("main_body", target_joint, interpolation, accuracy)

    def set_left_arm_joint(self, target_joint, interpolation=None, accuracy=0.008):
        return self._set_robot_group_joint("left_arm", target_joint, interpolation, accuracy)

    def set_right_arm_joint(self, target_joint, interpolation=None, accuracy=0.008):
        return self._set_robot_group_joint("right_arm", target_joint, interpolation, accuracy)

    # def get_head_EEF(self):
    #     """
    #
    #     :return: Head transformation with respect ot Back_Y frame
    #     """
    #     self.listener.waitForTransform('/Back_Y', '/Head', self.rospy.Time(), self.rospy.Duration(0.1))
    #     pose = self.listener.lookupTransform('/Back_Y', '/Head', self.rospy.Time(0))
    #     trans = pose[0]
    #     qua = pose[1]
    #
    #     return trans, qua


    def set_head_movement(self, neck_z, neck_x):
        """

        :param neck_z: Head horizontally rotation. Right(>0) - Left(<0)
        :param neck_x: Head vertically rotation. Up(>0) - Down(<0)
        :return:
        """
        # assert (type(neck_z) is float or type(neck_z) is int ) and (
        # type(neck_x) is float or type(neck_z) is int), 'Head joint is float number, please check'
        # assert -1 <= neck_x <= 0.8, 'Neck_x setting is out of permitted range, please check set_head_movement()'
        assert -0.87 <= neck_z <= 0.87, 'Neck_z setting is out of permitted range, please check set_head_movement()'

        if not self.rospy.is_shutdown():
            head_joint = JointState()
            head_joint.name = ["Neck_Z", "Neck_X"]
            head_joint.position = [neck_z, neck_x]
            self._ginger_pub.publish(head_joint)

    # def set_main_body_movement(self, back_z, back_x):
    #     """
    #
    #     :param back_z: Back side rotation. Right(>0) - Left(<0)
    #     :param back_x: Back rotation. Forward(>0)
    #     :return:
    #     """
    #     assert -1 <= back_z <= 1, 'Back_z is out of permitted range, please check set_main_body_movement()'
    #     assert 0 <= back_x <= 0.8, 'Back_x is out of permitted range, please check set_main_body_movement()'
    #
    #     if not self.rospy.is_shutdown():
    #         main_joint = JointState()
    #         main_joint.name = ["Back_Z", "Back_X"]
    #         main_joint.position = [back_z, back_x]
    #         self._ginger_pub.publish(main_joint)


    #################### Left Arm  ##################
    ### get-set pairs ######


    # def set_left_arm_joint(self, target_joint, interpolation=None,
    #                        accuracy=0.008):
    #     """
    #
    #     :param target_joint: Set left arm target.
    #     :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
    #     :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
    #     :return:
    #     """
    #
    #     assert type(target_joint) is list, 'Joint angle input is list'
    #     assert len(target_joint) == 7, 'list need to have size of 7'
    #
    #     if interpolation is not None:
    #         assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'
    #
    #     if not self.rospy.is_shutdown():
    #         left_arm = JointState()
    #         left_arm.name = self._left_arm_name
    #
    #         if interpolation is None or interpolation == 0:
    #             left_arm.position = target_joint
    #             self._ginger_pub.publish(left_arm)
    #
    #         else:
    #             cur_joint = []
    #             while True:
    #                 if len(cur_joint) != 7:
    #                     # print("get_joint list is empty")
    #                     cur_joint= self.get_left_arm_joint()
    #                     # print(cur_joint)
    #                 else:
    #                     break
    #             if len(cur_joint) != 7:
    #                 print("get_joint failed")
    #                 return False
    #
    #             unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]
    #
    #             for index in range(1, interpolation + 1):
    #                 tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
    #                 left_arm.position = tmp_target
    #                 while not self.rospy.is_shutdown():
    #                     error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, self.get_left_arm_joint())]
    #
    #                     if index == interpolation and np.all([e < accuracy for e in error_joint]):
    #                         print('Target arrived')
    #                         break
    #                     elif np.all([e < 5 * accuracy for e in error_joint]): # ??
    #                         break
    #                     self._ginger_pub.publish(left_arm)

    def get_left_arm_velocity(self):
        """

        :return: Get left arm velocity
        """
        velocity = self.joint_state.velocity[6: 13]

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
        left_arm = JointState()
        left_arm.name = self._left_arm_name

        while not self.rospy.is_shutdown():
            left_arm.velocity = target_velocity
            self._ginger_pub.publish(left_arm)
            error_vel = [np.abs(s - g) for (s, g) in zip(target_velocity, self.get_left_arm_velocity())]
            print(error_vel)
            if np.all([e < accuracy for e in error_vel]):
                print('Target velocity achieved')
                break

    def get_left_arm_current(self):
        """

        :return: Get left arm current
        """
        current = self.joint_state.effort[6: 13]

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
        left_arm = JointState()
        left_arm.name = self._left_arm_name


        while not self.rospy.is_shutdown():
            left_arm.effort = target_current
            self._ginger_pub.publish(left_arm)
            error_cur = [np.abs(s - g) for (s, g) in zip(target_current, self.get_left_arm_current())]
            print(error_cur)
            if np.all([e < accuracy for e in error_cur]):
                print('Target current achieved')
                break

    # def get_left_arm_EEF(self):
    #     """
    #
    #     :return: Get left arm End-Effector pose in translation, quaternion
    #     """
    #     self.listener.waitForTransform('/Back_Y', '/LeftEndEffector', self.rospy.Time(), self.rospy.Duration(0.1))
    #     pose = self.listener.lookupTransform('/Back_Y', '/LeftEndEffector', self.rospy.Time(0))
    #     trans = pose[0]
    #     qua = pose[1]
    #
    #     return trans, qua
    #
    # def set_left_arm_EEF(self, trans, quaternion):
    #     """
    #
    #     :param trans: <list> [x, y, z]
    #     :param quaternion: <list> [x, y, z, w]
    #     :return:
    #     """
    #     # translation = tuple([t * -1 for t in trans])
    #     self.broadcaster.sendTransform(trans,
    #                                    tuple(quaternion),
    #                                    rospy.Time.now(),
    #                                    '/LeftEndEffectorTarget',
    #                                    '/Back_Y')

    #################### Right Arm ##################





    # def set_right_arm_joint(self, target_joint, interpolation=None,
    #                         accuracy=0.008):
    #     """
    #
    #     :param target_joint: Set right arm target.
    #     :param interpolation: None and 0 -> no interpolation; >=1 -> interpolate target joint.
    #     :param accuracy: Target joint accuracy, 0.008 is roughly about 1 degree.
    #     :return:
    #     """
    #     assert type(target_joint) is list, 'Joint angle input is list'
    #
    #     if interpolation is not None:
    #         assert type(interpolation) is int and interpolation >= 0, 'Interpolation is positive number'
    #
    #     if not self.rospy.is_shutdown():
    #         right_arm = JointState()
    #         right_arm.name = self._right_arm_name
    #
    #         if interpolation is None or interpolation == 0:
    #             right_arm.position = target_joint
    #             self._ginger_pub.publish(right_arm)
    #
    #         else:
    #             cur_joint = []
    #             while True:
    #                 if len(cur_joint) != 7:
    #                     # print("get_joint list is empty")
    #                     cur_joint= self.get_right_arm_joint()
    #                     print(cur_joint)
    #                 else:
    #                     break
    #             if len(cur_joint) != 7:
    #                 print("get_joint failed")
    #                 return False
    #
    #             unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(target_joint, cur_joint)]
    #
    #             for index in range(1, interpolation + 1):
    #                 tmp_target = [cur + index * unit for (cur, unit) in zip(cur_joint, unit_step)]
    #                 right_arm.position = tmp_target
    #                 while not self.rospy.is_shutdown():
    #                     error_joint = [np.abs(tar - cur) for (tar, cur) in zip(tmp_target, self.get_right_arm_joint())]
    #
    #                     if index == interpolation and np.all([e < accuracy for e in error_joint]):
    #                         # print('Target arrived')
    #                         break
    #                     elif np.all([e < 5 * accuracy for e in error_joint]):
    #                         break
    #                     self._ginger_pub.publish(right_arm)



    def get_right_arm_velocity(self):
        """

        :return: Get right arm velocity
        """
        velocity = self.joint_state.velocity[13: 20]

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
        right_arm = JointState()
        right_arm.name = self._right_arm_name

        while not self.rospy.is_shutdown():
            right_arm.velocity = target_velocity
            self._ginger_pub.publish(right_arm)
            error_vel = [np.abs(s - g) for (s, g) in zip(target_velocity, self.get_right_arm_velocity())]
            if np.all([e < accuracy for e in error_vel]):
                print('Target velocity achieved')
                break

    def get_right_arm_current(self):
        """

        :return: Get right arm current
        """
        current = self.joint_state.effort[13: 20]

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
        right_arm = JointState()
        right_arm.name = self._right_arm_name

        while not self.rospy.is_shutdown():
            right_arm.current = target_current
            self._ginger_pub.publish(right_arm)
            error_cur = [np.abs(s - g) for (s, g) in zip(target_current, self.get_right_arm_current())]
            if np.all([e < accuracy for e in error_cur]):
                print('Target current achieved')
                break

    # def get_right_arm_EEF(self):
    #     """
    #
    #     :return: Get right arm End-Effector pose in translation, quaternion
    #     """
    #     self.listener.waitForTransform('/Back_Y', '/RightEndEffector', self.rospy.Time(), self.rospy.Duration(0.1))
    #     pose = self.listener.lookupTransform('/Back_Y', '/RightEndEffector', self.rospy.Time(0))
    #     trans = pose[0]
    #     qua = pose[1]
    #
    #     return trans, qua
    #
    # def set_right_arm_EEF(self, trans, quaternion):
    #     """
    #
    #     :param trans: <list> [x, y, z]
    #     :param quaternion: <list> [x, y, z, w]
    #     :return:
    #     """
    #     # translation = tuple([t * -1 for t in trans])
    #     trans = [trans[0] - 0.04, trans[1] + 0.05, trans[2]]
    #     self.broadcaster.sendTransform(trans,
    #                                    tuple(quaternion),
    #                                    rospy.Time.now(),
    #                                    '/RightEndEffectorTarget',
    #                                    '/Back_Y')

    def set_left_elbow(self, angle):
        """

        :param angle: Set left arm elbow angle for IK mode(4)
        :return:
        """
        assert type(angle) is float, 'ElbowAngle input is float'
        self.rospy.set_param('/ik/left_elbow_angle', angle)

    def set_right_elbow(self, angle):
        """

        :param angle: Set right arm elbow angle for IK mode(4)
        :return:
        """
        assert type(angle) is float, 'ElbowAngle input is float'
        self.rospy.set_param('/ik/right_elbow_angle', angle)

    ################### Hand #########################

    def set_left_hand_joint(self, joint):
        """

        :return: Set left hand joint
        """
        assert type(joint) is list, 'Joint input is list'
        left_hand = JointState()
        left_hand.name = self._left_hand_name
        self._ginger_pub.publish(left_hand)

    def set_right_hand_joint(self, joint):
        """

        :return: Set right hand joint
        """
        assert type(joint) is list, 'Joint input is list'
        right_hand = JointState()
        right_hand.name = self._right_hand_name
        self._ginger_pub.publish(right_hand)

    ########## utilis functions ###########################
    def go_zero(self):


        self.set_left_hand_joint([0, 0, 0, 0, 0])
        self.set_right_hand_joint([0, 0, 0, 0, 0])
        self.rospy.sleep(1.)


        self.set_main_body_joint([0, 0, 0, 0, 0, 0], interpolation=1)
        self.rospy.sleep(1.)

        self.set_left_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
        self.set_right_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)


    def go_sleep(self):

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


if __name__ == '__main__':

    robot = GingerTX2()
    # robot.go_zero()
    i = 2
    test_angle = 0.3
    target_joint= robot.get_right_arm_joint()
    target_joint[i] = test_angle
    robot.set_right_arm_joint(target_joint, interpolation=1, accuracy=0.01)
