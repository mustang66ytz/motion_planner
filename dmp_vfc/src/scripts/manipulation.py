#!/usr/bin/env python

from robots.ginger_TX2 import GingerTX2
from robots.ginger import Ginger
import warnings
import numpy as np
import rospy
from datetime import datetime, timedelta
import geometry_msgs.msg

from xr1controllervicap.srv import GraspingLinearService

Rockey = True #default: True
g_use_rocky_service = True
g_use_cv = True
g_first_action_need_drop = False

g_target_type = "beverage"
# g_target_type = "coffee"

g_grasp_height_offset_beverage = 0.13 # -0.07
g_drop_height_offset_beverage = -0.065

g_grasp_height_offset_beverage_cv = 0.095 # -0.06
g_drop_height_offset_beverage_cv = -0.10

g_grasp_height_offset_coffee = 0.085 # -0.03
g_drop_height_offset_coffee = -0.10

g_grasp_height_offset_coffee_cv = 0.085 # 0.01
g_drop_height_offset_coffee_cv = -0.09

g_base_height = None
g_upward_max_limit = None
g_upward_min_limit = None

g_marker_height_beverage = 0.08
g_marker_height_coffee = 0.08

g_grasp_y_offset_beverage = 0.0
g_grasp_z_offset_beverage = 0.0

g_grasp_y_offset_coffee = 0.0
g_grasp_z_offset_coffee = 0.0


def get_target_type():
    global g_target_type
    return g_target_type


def set_target_type(target_type):
    global g_target_type
    if not target_type in ["beverage", "coffee"]:
        rospy.loginfo("invalid target_type %s, assume to be coffee" % target_type)
        target_type = "coffee"
    g_target_type = target_type


def get_grasp_height_offset():
    if g_target_type == "coffee":
        if g_use_cv:
            return g_grasp_height_offset_coffee_cv
        else:
            return g_grasp_height_offset_coffee
    else:
        if g_use_cv:
            return g_grasp_height_offset_beverage_cv
        else:
            return g_grasp_height_offset_beverage


def get_drop_height_offset():
    if g_target_type == "coffee":
        if g_use_cv:
            return g_drop_height_offset_coffee_cv
        else:
            return g_drop_height_offset_coffee
    else:
        if g_use_cv:
            return g_drop_height_offset_beverage_cv
        else:
            return g_drop_height_offset_beverage


def get_base_height():
    global g_base_height
    return g_base_height


def get_upward_max_limit():
    global g_upward_max_limit
    return g_upward_max_limit


def get_upward_min_limit():
    global g_upward_min_limit
    return g_upward_min_limit


def get_marker_height():
    if g_target_type == "coffee":
        return g_marker_height_coffee
    else:
        return g_marker_height_beverage


def get_grasp_y_offset():
    if g_target_type == "coffee":
        return g_grasp_y_offset_coffee
    else:
        return g_grasp_y_offset_beverage


def get_grasp_z_offset():
    if g_target_type == "coffee":
        return g_grasp_z_offset_coffee
    else:
        return g_grasp_z_offset_beverage



def initial_preparation(robot):
    global g_first_action_need_drop

    rospy.loginfo("")
    rospy.loginfo("initial_preparation: Enter")
    rospy.loginfo("")

    drop_hand_if_needed(robot)

    cur_left_arm_joint = robot.get_left_arm_joint()
    rospy.loginfo("cur_left_arm_joint: %s" % cur_left_arm_joint)

    if g_first_action_need_drop:
        g_first_action_need_drop = False
        if cur_left_arm_joint[0] < -0.3:
            first_action(robot, drop_hand=True)

    robot.open_gripper('left')
    robot.open_gripper('right')
    robot.rospy.sleep(.5)
    robot.set_left_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
    rospy.loginfo("")
    rospy.loginfo("initial_preparation: left arm go zero")
    rospy.loginfo("")

    robot.set_right_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
    rospy.loginfo("")
    rospy.loginfo("initial_preparation: right arm go zero")
    rospy.loginfo("")

    robot.set_main_body_joint([0]*7, interpolation=1)
    rospy.loginfo("")
    rospy.loginfo("initial_preparation: main body go zero")
    rospy.loginfo("")

    # raw_input('Please say something')

    robot.set_main_body_joint([0, 1.5, 0, 0, 0, 0, 0], interpolation=None, send_times=10)
    robot.rospy.sleep(2)
    # robot.rospy.sleep(.5)
    # robot.set_head_movement(0, 0.5)
    if g_use_rocky_service:
        robot.set_main_body_joint([0, 1.57, 0, 0, 0.0, 0.5, 0], interpolation=None, send_times=2)
    else:
        robot.set_main_body_joint([0, 1.57, 0, 0, 0.2, 0.5, 0], interpolation=None, send_times=2)
    rospy.sleep(1)

def first_action(robot, drop_hand=False):
    global g_first_action_need_drop

    rospy.loginfo("")
    rospy.loginfo("first_action: Enter, drop_hand %s, g_first_action_need_drop %s" % (drop_hand, g_first_action_need_drop))
    rospy.loginfo("")

    cur_left_arm_joint = robot.get_left_arm_joint()
    rospy.loginfo("first_action:     cur_left_arm_joint: %s" % cur_left_arm_joint)

    is_valid = True

    if drop_hand and cur_left_arm_joint[0] > -0.3:
        is_valid = False

    if is_valid:
        first_segmentation = [[0] * 7,
                              [0.2229591807, 0.128731641521, -0.00444215269883, -0.38451785088, -0.0684538926683,
                               0, 0],
                              [0.521785319009, 0.272381728903, 0.180722111596, -0.926726796247, -0.0684645453127,
                               0, 0],
                              [0.750619360254, 0.449009001458, 0.120247049555, -1.19490711805, -0.0684751979571,
                               0, 0],
                              [1.216878041, 0.627135872975, -0.754750505671, -1.7930850569, -0.068560419112,
                               0, 0],
                              [0.81279508992, 0.698723203531, -1.01434479612, -1.79503449081, -0.0683793241578,
                               0, 0],
                              [0.0982112952757, 0.707819625549, -0.723176067783, -1.70617013155, -0.161984110164,
                               0, 0],
                              [-0.23253373376, 0.695820035451, -0.213724003829, -1.5188327278, -0.442297793898,
                               0, 0],
                              [-0.460696980902, 0.620706211476, 0.105908590244, -1.42564339493, -0.517505463092,
                               0, 0],
                              [-0.45, 0.7, 0.25, - 1.41, - 0.51, 0, 0]]

        if drop_hand:
            first_segmentation.reverse()

        for index, each_joint in enumerate(first_segmentation):
            robot.rospy.loginfo("index: %s" % index)
            if index == 0:
                robot.set_left_arm_joint(each_joint, interpolation=None, send_times=20)
                # robot.rospy.sleep(1.5)
            else:
                robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
                robot.rospy.sleep(.7)
        # robot.rospy.sleep(2)
        if drop_hand:
            g_first_action_need_drop = False
            robot.set_main_body_joint([0, 0, 0, 0, 0, 0, 0], interpolation=None, send_times=10)
        else:
            rospy.sleep(1)
            g_first_action_need_drop = True

    robot.rospy.loginfo("Leave: is_valid %s, g_first_action_need_drop %s" % (is_valid, g_first_action_need_drop))



    ## error handling case

    # for index, each_joint in enumerate(reversed(first_segmentation)):
    #     robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
    #     robot.rospy.sleep(.6)


def move_to(robot, tolerance=0.03):
    g_first_action_need_drop = False

    rospy.loginfo("")
    rospy.loginfo("move_to: Enter")
    rospy.loginfo("")

    robot.rate.sleep()
    robot.open_gripper('left')

    robot.rospy.loginfo("Start Grasping with move_to()")

    use_rocky_service = g_use_rocky_service
    grasp_ok = False

    rospy.loginfo("move_to: use_rocky_service %s" % use_rocky_service)

    if use_rocky_service:
        # target_pos = get_target_pos(robot, step_timeout=20,
        #            base_height=None, upward_max_limit=None, upward_min_limit=None)
        grasp_height_offset = get_grasp_height_offset()
        rospy.loginfo("grasp_height_offset %s" % grasp_height_offset)
        target_pos = [grasp_height_offset, 0, 0]
        if target_pos:
            rospy.wait_for_service('XR1/GLPT')
            grasping_service = rospy.ServiceProxy('XR1/GLPT', GraspingLinearService)
            try:
                rsp_seq = 0
                grasping_service_rsp = grasping_service(NewTarget=True,
                                                        Termination=False,
                                                        Grasp_or_Drop=True,
                                                        x=target_pos[0],
                                                        y=target_pos[1],
                                                        z=target_pos[2])
                rsp_seq += 1
                rospy.loginfo("move_to: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
                if not grasping_service_rsp.isAccepted:
                    rospy.logerr("move_to: isAccepted False?")
                else:
                    target_time = datetime.now() + timedelta(seconds=15)
                    while datetime.now() < target_time:
                        rospy.sleep(0.5)
                        grasping_service_rsp = grasping_service(NewTarget=False,
                                                                Termination=False,
                                                                Grasp_or_Drop=True,
                                                                x=target_pos[0],
                                                                y=target_pos[1],
                                                                z=target_pos[2])
                        rsp_seq += 1
                        rospy.loginfo("move_to: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
                        if grasping_service_rsp.inProgress:
                            continue
                        else:
                            rospy.loginfo("")
                            rospy.loginfo("")
                            rospy.loginfo("move_to: grasp_ok")
                            rospy.loginfo("")
                            rospy.loginfo("")
                            grasp_ok = True
                            break
                    if not grasp_ok:
                        grasping_service_rsp = grasping_service(NewTarget=False,
                                                                Termination=True,
                                                                Grasp_or_Drop=True,
                                                                x=target_pos[0],
                                                                y=target_pos[1],
                                                                z=target_pos[2])
                        rsp_seq += 1
                        rospy.loginfo("move_to: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
            except Exception as e:
                rospy.logerr("move_to: met exception %s" % e)

            robot.set_left_arm_mode(1)
    else:
        if Rockey:
            robot.set_left_arm_mode(4)
            robot.set_right_arm_mode(4)


        current_pos, target_quaterenion = robot.get_left_arm_EEF()

        offset = get_offset(robot)
        print(offset)


        grasp_point = [current_pos[0] + offset[0],
                       current_pos[1] + offset[1],
                       current_pos[2] + offset[2]]

        robot.broadcaster.sendTransform(tuple(grasp_point),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        grasp_point1 = [current_pos[0] + offset[0]/3.0,
                       current_pos[1] + offset[1]/3.0,
                       current_pos[2] + offset[2]/3.0]



        grasp_point2 = [current_pos[0] + offset[0]*2/3.0,
                       current_pos[1] + offset[1]*2/3.0,
                       current_pos[2] + offset[2]*2/3.0]

        grasp_point3 = [current_pos[0] + offset[0]*3/4.0,
                       current_pos[1] + offset[1]*3/4.0,
                       current_pos[2] + offset[2]*3/4.0]

        grasp_point4 = [current_pos[0] + offset[0]*4/5.0,
                       current_pos[1] + offset[1]*4/5.0,
                       current_pos[2] + offset[2]*4/5.0]

        grasp_point5 = [current_pos[0] + offset[0]*9/10.0,
                       current_pos[1] + offset[1]*9/10.0,
                       current_pos[2] + offset[2]*9/10.0]

        grasp_point6 = [current_pos[0] + offset[0]*9/10.0,
                       current_pos[1] + offset[1]*9/10.0,
                       current_pos[2] + offset[2]*9/10.0 + 0.07]

        # while True:
        #
        #     robot.broadcaster.sendTransform(tuple(grasp_point1),
        #                                     tuple(target_quaterenion),
        #                                     robot.rospy.Time.now(),
        #                                     '/Delta1',
        #                                     '/Back_Y')
        #
        #     robot.broadcaster.sendTransform(tuple(grasp_point2),
        #                                     tuple(target_quaterenion),
        #                                     robot.rospy.Time.now(),
        #                                     '/Delta2',
        #                                     '/Back_Y')
        #
        #     robot.broadcaster.sendTransform(tuple(grasp_point3),
        #                                     tuple(target_quaterenion),
        #                                     robot.rospy.Time.now(),
        #                                     '/Delta3',
        #                                     '/Back_Y')
        #
        #     robot.broadcaster.sendTransform(tuple(grasp_point4),
        #                                     tuple(target_quaterenion),
        #                                     robot.rospy.Time.now(),
        #                                     '/Delta4',
        #                                     '/Back_Y')
        #
        #     robot.broadcaster.sendTransform(tuple(grasp_point),
        #                                     tuple(target_quaterenion),
        #                                     robot.rospy.Time.now(),
        #                                     '/Delta',
        #                                     '/Back_Y')
        #
        #     try:
        #         robot.listener.waitForTransform('/LeftEndEffector', '/Center', robot.rospy.Time(),
        #                                         robot.rospy.Duration(0.1))
        #         offset = robot.listener.lookupTransform('/LeftEndEffector', '/Center', robot.rospy.Time(0))
        #
        #         robot.broadcaster.sendTransform(tuple(offset[0]),
        #                                         tuple([0, 0, 0, 1]),
        #                                         robot.rospy.Time.now(),
        #                                         '/Targett',
        #                                         '/Delta')
        #
        #     except:
        #         warnings.warn('Can not receive tf transform from Grasp_point', UserWarning)
        #         continue



        # while True:

        current_pos, target_quaterenion = robot.get_left_arm_EEF()

        # remember it is calculating marker frame and convert it back to LeftEEF

        robot.broadcaster.sendTransform(tuple(grasp_point1),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        # robot.rospy.sleep(0.5)
        _, cur_quaterenion = robot.get_left_arm_EEF()
        robot.set_left_arm_EEF(grasp_point1, cur_quaterenion)
        # robot.rospy.sleep(2)


        robot.broadcaster.sendTransform(tuple(grasp_point2),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        robot.rospy.sleep(0.5)
        _, cur_quaterenion = robot.get_left_arm_EEF()
        robot.set_left_arm_EEF(grasp_point2, cur_quaterenion)
        robot.rospy.sleep(0.5)


        robot.broadcaster.sendTransform(tuple(grasp_point3),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        _, cur_quaterenion = robot.get_left_arm_EEF()
        robot.set_left_arm_EEF(grasp_point3, cur_quaterenion)
        robot.rospy.sleep(0.5)

        robot.broadcaster.sendTransform(tuple(grasp_point4),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        _, cur_quaterenion = robot.get_left_arm_EEF()
        robot.set_left_arm_EEF(grasp_point4, cur_quaterenion)
        robot.rospy.sleep(0.5)

        robot.broadcaster.sendTransform(tuple(grasp_point5),
                                        tuple(target_quaterenion),
                                        robot.rospy.Time.now(),
                                        '/Delta',
                                        '/Back_Y')

        _, cur_quaterenion = robot.get_left_arm_EEF()
        robot.set_left_arm_EEF(grasp_point5     , cur_quaterenion)
        robot.rospy.sleep(.5)

        # robot.broadcaster.sendTransform(tuple(grasp_point),
        #                                 tuple(target_quaterenion),
        #                                 robot.rospy.Time.now(),
        #                                 '/Delta',
        #                                 '/Back_Y')
        #
        # # cur_trans, cur_qua = robot.get_left_arm_EEF()
        # # error = [t - c for (t, c) in zip(grasp_point, cur_trans)]
        # #
        # # if all(np.abs(error) < tolerance):
        # #     robot.rospy.loginfo('Successfully move to target !!!')
        # #     break
        #
        # robot.rospy.sleep(0.5)
        # robot.set_left_arm_EEF(grasp_point, target_quaterenion)
        # robot.rospy.sleep(.5)



        robot.rospy.loginfo('Start Grasping Coffee')
        robot.rospy.sleep(0.1)

        robot.close_gripper('left', 15)
        robot.rospy.sleep(0.5)

        robot.set_left_arm_mode(1)
        robot.set_left_arm_mode(1)

        robot.rospy.sleep(0.1)
        cur_left_arm_joint = robot.get_left_arm_joint()
        robot.rospy.loginfo('cur_left_arm_joint %s' % cur_left_arm_joint)
        cur_left_arm_joint[3] -= 0.3
        robot.set_left_arm_joint(cur_left_arm_joint, interpolation=None, send_times=10)

        grasp_ok = True

    rospy.loginfo("")
    rospy.loginfo("move_to: Leave")
    rospy.loginfo("")

    return grasp_ok

def turn_back(robot):
    rospy.loginfo("")
    rospy.loginfo("turn_back: Enter")
    rospy.loginfo("")

    cur_main_body_joint = robot.get_main_body_joint()
    cur_main_body_joint[1] = 0
    cur_main_body_joint[4] = 0
    robot.set_main_body_joint(cur_main_body_joint, interpolation=None, send_times=2)
    robot.rospy.sleep(2)
    # robot.set_left_arm_joint([-0.98843981546, 0.129958370501, 0.45277999595, -1.1848653018, -0.444449628059, 0.297497517187, -0.515524095814], interpolation=None, send_times=10)
    # robot.rospy.sleep(.3)
    # robot.set_left_arm_joint([-1.1, 0.07, -0.06, -0.6, 0.0, -0.3, 0.0], interpolation=None, send_times=10)

    rospy.loginfo("")
    rospy.loginfo("turn_back: Leave")
    rospy.loginfo("")

def last_action(robot):
    rospy.loginfo("")
    rospy.loginfo("last_action: Enter")
    rospy.loginfo("")

    use_rocky_service = g_use_rocky_service

    if use_rocky_service:
        target_time = datetime.now() + timedelta(seconds=10)
        while datetime.now() < target_time:
            cur_main_body_joint = robot.get_main_body_joint()
            if cur_main_body_joint[1] < 0.01:
                rospy.loginfo("last_action: cur_main_body_joint %s" % cur_main_body_joint)
                rospy.sleep(0.5)
                break
            rospy.sleep(0.2)

        drop_height_offset = get_drop_height_offset()
        rospy.loginfo("drop_height_offset %s" % drop_height_offset)
        target_pos = [drop_height_offset, 0, 0]
        rospy.wait_for_service('XR1/GLPT')
        grasping_service = rospy.ServiceProxy('XR1/GLPT', GraspingLinearService)
        try:
            drop_ok = False

            rsp_seq = 0
            grasping_service_rsp = grasping_service(NewTarget=True,
                                                    Termination=False,
                                                    Grasp_or_Drop=False,
                                                    x=target_pos[0],
                                                    y=target_pos[1],
                                                    z=target_pos[2])
            rsp_seq += 1
            rospy.loginfo("last_action: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
            if not grasping_service_rsp.isAccepted:
                rospy.logerr("last_action: isAccepted False?")
            else:
                target_time = datetime.now() + timedelta(seconds=15)
                while datetime.now() < target_time:
                    rospy.sleep(0.5)
                    grasping_service_rsp = grasping_service(NewTarget=False,
                                                            Termination=False,
                                                            Grasp_or_Drop=False,
                                                            x=target_pos[0],
                                                            y=target_pos[1],
                                                            z=target_pos[2])
                    rsp_seq += 1
                    rospy.loginfo("last_action: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
                    if grasping_service_rsp.inProgress:
                        continue
                    else:
                        rospy.loginfo("")
                        rospy.loginfo("")
                        rospy.loginfo("last_action: drop_ok")
                        rospy.loginfo("")
                        rospy.loginfo("")
                        drop_ok = True
                        break
            if not drop_ok:
                grasping_service_rsp = grasping_service(NewTarget=False,
                                                        Termination=True,
                                                        Grasp_or_Drop=False,
                                                        x=target_pos[0],
                                                        y=target_pos[1],
                                                        z=target_pos[2])
                rsp_seq += 1
                rospy.loginfo("last_action: rsp_seq %s, grasping_service_rsp %s" % (rsp_seq, grasping_service_rsp))
        except Exception as e:
            rospy.logerr("last_action: met exception %s" % e)

        robot.set_left_arm_mode(1)

        robot.set_left_hand_joint([0.0, 0.5, 0.6, 0.6, 0.9])

        drop_hand = [
            [0.415, 0.0, -0.004, -2, -0.091, 0.056, -0.124],
            [0.718, 0.0, -0.004, -2, -0.091, 0.056, -0.124],
            [0.699, 0.0, 0.178, -1.3, -0.091, -0.294, -0.232],
            [0.435, 0.0, 0.082, -0.982, -0.094, -0.391, -0.127],
            [-0.042, 0.0, 0.082, -0.023, -0.095, -0.181, 0.051],
            [0] * 7]
    else:
        robot.set_left_arm_joint([-1.39, 0.071, -0.068, -0.366, 0.0811, -0.072, -0.0347], interpolation=None,
                                 send_times=10)
        robot.rospy.sleep(0.5)

        for i in range(0, 7):
            robot.close_gripper('left', 10 - i)
            robot.rospy.sleep(.1)

        robot.open_gripper('left')

        robot.rospy.sleep(1)

        drop_hand = [
            [-0.855999326747, 0.156023414689, -0.105865979667, -1.34474781332, -0.0949470191959, 0.0431275392119,
             -0.248542633256],
            [-0.651247790189, 0.103065395303, -0.112119081907, -1.36319759368, -0.09112271987, 0.0842201993089,
             -0.0951515329581],
            [-0.337211433835, 0.00763570936815, -0.13059076723, -1.74563817891, -0.0911759830919, 0.101998681002,
             -0.112293203917],
            [0.00828780932827, 0.00766796898745, -0.00433562625522, -2.10318353427, -0.0912718568911, 0.0559885559612,
             -0.123549575879],
            [0.414743335258, 0.00752825705993, 0.0830480154439, -2.09420335507, -0.0950322403508, -0.298194777953,
             -0.23608321686],
            [0.71849634239, 0.0296495965981, 0.178378529837, -1.77568928865, -0.0912505516024, -0.369255861941,
             -0.306873031628],
            [0.699807297349, 0.0290524763503, 0.178687456523, -1.53136023757, -0.091591436222, -0.294584943919,
             -0.232176656919],
            [0.435298621642, 0.0291271280846, 0.0828882257784, -0.982322947171, -0.0947126610199, -0.391147072022,
             -0.127817813715],
            [-0.042290228296, 0.00589414765663, 0.0827710466905, -0.02303101711, -0.0950535456395, -0.1812443586,
             0.0510614919652],
            [0] * 7]

    for index, each_joint in enumerate(drop_hand):
        rospy.loginfo("index: %s" % index)
        if index == 0:
            robot.set_left_arm_joint(each_joint, interpolation=1, send_times=5)
        else:
            robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
        robot.rospy.sleep(.3)

    robot.open_gripper('left')
    robot.set_head_movement(0, 0)

    rospy.loginfo("")
    rospy.loginfo("last_action: Leave")
    rospy.loginfo("")


def drop_hand_if_needed(robot):
    cur_main_body_joint = robot.get_main_body_joint()
    cur_left_arm_joint = robot.get_left_arm_joint()

    if cur_left_arm_joint[3] < -1:
        rospy.loginfo("")
        rospy.loginfo("")
        rospy.loginfo("drop_hand_if_needed: need drop hand, cur_left_arm_joint %s" % cur_left_arm_joint)
        rospy.loginfo("drop_hand_if_needed: cur_main_body_joint %s" % cur_main_body_joint)
        rospy.loginfo("")
        rospy.loginfo("")
        if cur_main_body_joint[1] > 1:
            # in state: turn left
            drop_hand = [[0] * 7,
                                  [0.2229591807, 0.128731641521, -0.00444215269883, -0.38451785088, -0.0684538926683,
                                   0, 0],
                                  [0.521785319009, 0.272381728903, 0.180722111596, -0.926726796247, -0.0684645453127,
                                   0, 0],
                                  [0.750619360254, 0.449009001458, 0.120247049555, -1.19490711805, -0.0684751979571,
                                   0, 0],
                                  [1.216878041, 0.627135872975, -0.754750505671, -1.7930850569, -0.068560419112,
                                   0, 0],
                                  [0.81279508992, 0.698723203531, -1.01434479612, -1.79503449081, -0.0683793241578,
                                   0, 0],
                                  [0.0982112952757, 0.707819625549, -0.723176067783, -1.70617013155, -0.161984110164,
                                   0, 0],
                                  [-0.23253373376, 0.695820035451, -0.213724003829, -1.5188327278, -0.442297793898,
                                   0, 0],
                                  [-0.460696980902, 0.620706211476, 0.105908590244, -1.42564339493, -0.517505463092,
                                   0, 0],
                                  [-0.45, 0.7, 0.25, - 1.41, - 0.51, 0, 0]]

            drop_hand.reverse()
        else:
            drop_hand = [
                [0.415, 0.0, -0.004, -2, -0.091, 0.056, -0.124],
                [0.718, 0.0, -0.004, -2, -0.091, 0.056, -0.124],
                [0.699, 0.0, 0.178, -1.3, -0.091, -0.294, -0.232],
                [0.435, 0.0, 0.082, -0.982, -0.094, -0.391, -0.127],
                [-0.042, 0.0, 0.082, -0.023, -0.095, -0.181, 0.051],
                [0] * 7]

        for index, each_joint in enumerate(drop_hand):
            if index == 0:
                robot.set_left_arm_joint(each_joint, interpolation=1, send_times=5)
            else:
                robot.set_left_arm_joint(each_joint, interpolation=None, send_times=5)
            robot.rospy.sleep(.5)


def move_to_linear(robot, target_pose, tolerance=0.01):
    # rrr = [0.135356627603, 0.86709054426, 0.092532487246, 0.470393806582]   # Right Hand
    llll = [-0.144967529685, 0.822168555019, -0.0292108388561, 0.549699947182]  # Left Hand

    robot.set_left_arm_joint([-1, 0.8, 0, -1, -0.5, 0, 0], interpolation=1)
    robot.rate.sleep()
    robot.set_left_hand_joint(([0, 0, 0, 0, 0]))

    robot.rospy.loginfo("Start Grasping with move_to_linear()")

    if Rockey:
        robot.set_left_arm_mode(4)
        robot.set_right_arm_mode(4)

    target_translation = target_pose[0]
    interpolation = 20

    grasp_point = [target_translation[0] + 0.1,
                   target_translation[1] - 0,
                   target_translation[2] - 0]

    cur_position, q = robot.get_left_arm_EEF()

    unit_step = [(tar - cur) / interpolation for (tar, cur) in zip(grasp_point, cur_position)]

    for index in range(1, interpolation + 1):
        tmp_target = [cur + index * unit for (cur, unit) in zip(cur_position, unit_step)]

        while not robot.rospy.is_shutdown():
            cur_position, cur_qua = robot.get_left_arm_EEF()

            robot.broadcaster.sendTransform(tuple(tmp_target),
                                            tuple(cur_qua),
                                            robot.rospy.Time.now(),
                                            '/Grasping_Point',
                                            '/Back_Y')


            error = [t - c for (t, c) in zip(tmp_target, cur_position)]

            if index == interpolation and np.all([e < 3 * tolerance for e in error]):
                robot.rospy.loginfo('Successfully move to target !!!')
                break
            elif np.all([e < 2 * tolerance for e in error]):
                break
            robot.set_left_arm_EEF(tmp_target, q)

    return

tf_selection = 0 #1:HARI, 0:Realsense_TX2

def round_float_list(float_list, ndigits=3):
    rounded_float_list = [round(cur_float, ndigits) for cur_float in float_list]
    return rounded_float_list

def get_offset(robot, step_timeout=20):
    rospy.loginfo("get_offset: Enter")

    offset = None
    last_tmp_offset = None
    same_cnt = 0

    time_deadline = datetime.now() + timedelta(seconds=step_timeout)

    while offset is None:
        try:
            if datetime.now() >= time_deadline:
                rospy.loginfo("get_offset: Leave timeout, no offset got")
                return [0, 0, 0]

            robot.listener.waitForTransform('/Back_Y', '/Center', robot.rospy.Time(),
                                            robot.rospy.Duration(0.5))
            offset1 = robot.listener.lookupTransform('/Back_Y', '/Center', robot.rospy.Time(0))
            offset1 = offset1[0]

            robot.listener.waitForTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(),
                                            robot.rospy.Duration(0.5))
            offset2 = robot.listener.lookupTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(0))
            offset2 = offset2[0]

            tmp_offset = [offset2[0] - offset1[0],
                      offset2[1] - offset1[1],
                      offset2[2] - offset1[2]]
            tmp_offset = round_float_list(tmp_offset)
            height_offset = 0.0
            tmp_offset[0] += height_offset
            if not last_tmp_offset is None:
                rospy.loginfo("")
                rospy.loginfo("offset2 %s, offset1 %s" % (offset2, offset1))
                rospy.loginfo("tmp_offset %s, last_tmp_offset %s" % (tmp_offset, last_tmp_offset))
                diff = np.array(last_tmp_offset) - np.array(tmp_offset)
                dist = np.linalg.norm(diff)
                if dist < 0.01:
                    same_cnt += 1
                    rospy.loginfo("same_cnt: %s, dist %.3f" % (same_cnt, dist))
                else:
                    rospy.loginfo("same_cnt: %s, dist %.3f" % (same_cnt, dist))
                    same_cnt = 0
                rospy.loginfo("")
                upward_max_limit = 0.05
                upward_min_limit = -0.03
                if tmp_offset[0] > upward_max_limit:
                    rospy.logwarn("x (upward) is larger than %s ?? tmp_offset %s" % (upward_max_limit, tmp_offset))
                elif tmp_offset[0] < upward_min_limit:
                    rospy.logwarn("x (upward) is smaller than %s ?? tmp_offset %s" % (upward_min_limit, tmp_offset))
                elif same_cnt > 2:
                    offset = tmp_offset
                    break
            last_tmp_offset = tmp_offset
            rospy.sleep(0.2)
        except Exception as e:
            # should move head to search around?
            rospy.logwarn("get_offset: Exception %s" % e)
            last_tmp_offset = None
            continue

    rospy.loginfo("get_offset: Leave with offset %s" % offset)

    return offset

def setup_marker_pos(robot, br_static):
    base_height = get_base_height()
    upward_max_limit = get_upward_max_limit()
    upward_min_limit = get_upward_min_limit()

    rospy.loginfo("")
    rospy.loginfo("setup_marker_pos: Enter, base_height %s, upward_max_limit %s, upward_min_limit %s" %
                  (base_height, upward_max_limit, upward_min_limit))
    rospy.loginfo("")

    set_ok = False
    height_ok = True

    try:
        if True:
            robot.listener.waitForTransform('/Back_Y', '/Marker_31', robot.rospy.Time(),
                                            robot.rospy.Duration(0.5))
            cv_transform = robot.listener.lookupTransform('/Back_Y', '/Marker_31', robot.rospy.Time(0))
            cv_translation = round_float_list(cv_transform[0])
            cv_rotation = round_float_list(cv_transform[1])

            rospy.loginfo("")
            rospy.loginfo("setup_marker_pos: Enter, cv_translation %s, cv_rotation %s" %
                          (cv_translation, cv_rotation))
            rospy.loginfo("")

            set_ok = True

        if False:
            robot.listener.waitForTransform('/Back_Y', '/Marker_31_cv', robot.rospy.Time(),
                                            robot.rospy.Duration(0.5))
            cv_transform = robot.listener.lookupTransform('/Back_Y', '/Marker_31_cv', robot.rospy.Time(0))
            cv_translation = round_float_list(cv_transform[0])
            cv_rotation = round_float_list(cv_transform[1])

            rospy.loginfo("")
            rospy.loginfo("setup_marker_pos: Enter, cv_translation %s, cv_rotation %s" %
                          (cv_translation, cv_rotation))
            rospy.loginfo("")

            if base_height is not None:
                height_diff = cv_translation[0] - base_height

                if upward_max_limit is not None:
                    if height_diff > upward_max_limit:
                        rospy.logwarn(
                            "height_diff %s is larger than %s" % (height_diff, upward_max_limit))
                        height_ok = False

                if upward_min_limit is not None:
                    if height_diff < upward_min_limit:
                        rospy.logwarn(
                            "height_diff %s is smaller than %s" % (height_diff, upward_min_limit))
                        height_ok = False

            if True:
                grasp_x_offset = get_marker_height()
                grasp_y_offset = get_grasp_y_offset()
                grasp_z_offset = get_grasp_z_offset()

                cv_translation_new = [grasp_x_offset,
                                      cv_translation[1] + grasp_y_offset,
                                      cv_translation[2] + grasp_z_offset]

                rospy.loginfo("")
                rospy.loginfo("cv_translation_new: %s" % cv_translation_new)
                rospy.loginfo("")

                static_transformStamped = geometry_msgs.msg.TransformStamped()
                static_transformStamped.header.stamp = rospy.Time.now()
                static_transformStamped.header.frame_id = "Back_Y"
                static_transformStamped.child_frame_id = "Marker_31"
                static_transformStamped.transform.translation.x = float(cv_translation_new[0])
                static_transformStamped.transform.translation.y = float(cv_translation_new[1])
                static_transformStamped.transform.translation.z = float(cv_translation_new[2])
                static_transformStamped.transform.rotation.x = cv_rotation[0]
                static_transformStamped.transform.rotation.y = cv_rotation[1]
                static_transformStamped.transform.rotation.z = cv_rotation[2]
                static_transformStamped.transform.rotation.w = cv_rotation[3]
                br_static.sendTransform(static_transformStamped)

                set_ok = True
    except Exception as e:
        # should move head to search around?
        rospy.logwarn("setup_marker_pos: Exception %s" % e)

    rospy.loginfo("")
    rospy.loginfo("setup_marker_pos: Leave set_ok %s, height_ok %s" % (set_ok, height_ok))
    rospy.loginfo("")

    return set_ok


def get_target_pos(robot, step_timeout=20,
                   base_height=None, upward_max_limit=None, upward_min_limit=None):
    rospy.loginfo("get_target_pos: Enter")

    target_pos = None
    last_tmp_target_pos = None
    same_cnt = 0

    time_deadline = datetime.now() + timedelta(seconds=step_timeout)

    while target_pos is None:
        try:
            if datetime.now() >= time_deadline:
                rospy.loginfo("get_target_pos: Leave timeout, no target_pos got")
                return None

            robot.listener.waitForTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(),
                                            robot.rospy.Duration(0.5))
            offset2 = robot.listener.lookupTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(0))
            offset2 = offset2[0]

            tmp_target_pos = round_float_list(offset2)
            height_offset = 0.0
            tmp_target_pos[0] += height_offset
            if not last_tmp_target_pos is None:
                rospy.loginfo("")
                rospy.loginfo("tmp_target_pos %s, last_tmp_target_pos %s" % (tmp_target_pos, last_tmp_target_pos))
                diff = np.array(last_tmp_target_pos) - np.array(tmp_target_pos)
                dist = np.linalg.norm(diff)
                if dist < 0.01:
                    same_cnt += 1
                    rospy.loginfo("same_cnt: %s, dist %.3f" % (same_cnt, dist))
                else:
                    rospy.loginfo("same_cnt: %s, dist %.3f" % (same_cnt, dist))
                    same_cnt = 0
                rospy.loginfo("")
                height_ok = True
                if base_height is not None:
                    height_diff = tmp_target_pos[0] - base_height

                    if upward_max_limit is not None:
                        if height_diff > upward_max_limit:
                            rospy.logwarn(
                                "height_diff %s is larger than %s" % (height_diff, upward_max_limit))
                            height_ok = False

                    if upward_min_limit is not None:
                        if height_diff < upward_min_limit:
                            rospy.logwarn(
                                "height_diff %s is smaller than %s" % (height_diff, upward_min_limit))
                            height_ok = False

                if height_ok and same_cnt > 2:
                    target_pos = tmp_target_pos
                    break
            last_tmp_target_pos = tmp_target_pos
            rospy.sleep(0.2)
        except Exception as e:
            # should move head to search around?
            rospy.logwarn("get_target_pos: Exception %s" % e)
            last_tmp_target_pos = None
            continue

    rospy.loginfo("get_target_pos: Leave with target_pos %s" % target_pos)

    return target_pos

def get_offset_old(robot):
    offset = None

    while offset is None:
        try:
            robot.listener.waitForTransform('/Back_Y', '/Center', robot.rospy.Time(),
                                            robot.rospy.Duration(0.1))
            offset1 = robot.listener.lookupTransform('/Back_Y', '/Center', robot.rospy.Time(0))
            offset1 = offset1[0]

            robot.listener.waitForTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(),
                                            robot.rospy.Duration(0.1))
            offset2 = robot.listener.lookupTransform('/Back_Y', '/Grasp_point', robot.rospy.Time(0))
            offset2 = offset2[0]

            offset = [offset2[0] - offset1[0],
                      offset2[1] - offset1[1],
                      offset2[2] - offset1[2]]

        except:
            warnings.warn('Can not receive tf transform from Grasp_point', UserWarning)
            continue
    # print(offset[0])
    return offset
    # return offset[0]


def get_target(robot):
    target_pose = None

    while target_pose is None:
        try:
            if tf_selection == 0:
                robot.listener.waitForTransform('/Back_Y', 'Grasp_point', robot.rospy.Time(),
                                                robot.rospy.Duration(0.1))
                target_pose = robot.listener.lookupTransform('/Back_Y', 'Grasp_point', robot.rospy.Time(0))

                # # robot.listener.waitForTransform('/Back_Y', '/LeftEndEffector', robot.rospy.Time(),
                # #                                 robot.rospy.Duration(0.1))
                # g_LeftEndEffector_Back_Y = robot.listener.lookupTransform('/Back_Y', '/LeftEndEffector', robot.rospy.Time(0))
                # g_LeftEndEffector_LeftGraspFrame = robot.listener.lookupTransform('/LeftGraspFrame', '/LeftEndEffector', robot.rospy.Time(0))

            #     robot.broadcaster.sendTransform(tuple(marker_pose[0]),
            #                                     tuple(g_LeftEndEffector_Back_Y[1]),
            #                               # (0, 0, 0, 1),
            #                               robot.rospy.Time.now(),
            #                               '/Mock_Marker_',
            #                               '/Back_Y')
            #
            #     # robot.listener.waitForTransform('/Back_Y', '/LeftEndEffector', robot.rospy.Time(),
            #     #                                 robot.rospy.Duration(0.1))
            #     robot.broadcaster.sendTransform(tuple(g_LeftEndEffector_LeftGraspFrame[0]),
            #                                     tuple(g_LeftEndEffector_LeftGraspFrame[1]),
            #                               robot.rospy.Time.now(),
            #                               '/TargetFrame',
            #                               '/Mock_Marker_')
            #
            #     robot.listener.waitForTransform('/Back_Y', '/TargetFrame', robot.rospy.Time(),
            #                                     robot.rospy.Duration(0.1))
            #     target_pose = robot.listener.lookupTransform('/Back_Y', '/TargetFrame', robot.rospy.Time(0))
            #
            # elif tf_selection == 1:
            #     robot.listener.waitForTransform('/Back_Y', 'Toy_0' , robot.rospy.Time(),
            #                                     robot.rospy.Duration(0.1))
            #     target_pose = robot.listener.lookupTransform('/LeftGraspFrame', 'Toy_0', robot.rospy.Time(0))
        except:
            warnings.warn('Can not receive tf transform from Marker_', UserWarning)
            continue

    return target_pose

# put into util function
def build_relative_offset(x_off, y_off, z_off):
    return np.matrix([[1, 0, 0, 0.03+x_off],
                      [0, 1, 0, 0+y_off],
                      [0, 0, 1, 0+z_off],
                      [0, 0, 0, 1]])
def manipulate(robot=None, target_pose=None):
    rospy.loginfo("")
    rospy.loginfo("manipulate: Enter")
    rospy.loginfo("")

    if robot == None:
        robot = Ginger()
        rospy.sleep(4)

    # offset = get_offset(robot)
    # rospy.loginfo("offset %s" % offset)
    # exit(-1)

    robot.open_gripper('left')

    robot.rate.sleep()

    if True:
        # # Set the control mode
        robot.set_main_body_mode(1)
        robot.set_left_hand_mode(1)
        robot.set_right_hand_mode(1)
        robot.set_left_arm_mode(1)
        robot.set_right_arm_mode(1)

    rospy.loginfo("Ginger stand up")

    while True:
        # robot.set_left_arm_joint([-1, 0.01 + delta, 0.01 + delta, -1, 0.01, 0.01, 0.01], interpolation=1)
        # option = raw_input('Press 1 : Move To in real time. Press 2: Move Linear')
        option = 1

        if option == 1:
            initial_preparation(robot)
            first_action(robot)
            # robot.rospy.sleep(3)
            grasp_ok = move_to(robot)
            if not grasp_ok:
                first_action(robot, drop_hand=True)
            else:
                turn_back(robot)
                last_action(robot)

            exit(0)

        elif option == 2:
            move_to_linear(robot)

        elif option == 3:

            # remember it is calculating marker frame and convert it back to LeftEEF
            robot.broadcaster.sendTransform(tuple(target_pose[0]),
                                            tuple(target_pose[1]),
                                            robot.rospy.Time.now(),
                                            '/TargetFrame',
                                            '/Back_Y')
            flag, path = robot.set_left_arm_EEF_linear(target_pose[0], target_pose[1])
            if flag:
                break



        else:
            warnings.warn('Invalid user input, please type again.', SyntaxWarning)
            continue


if __name__ == '__main__':
    manipulate()