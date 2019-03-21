#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import std_msgs.msg
import geometry_msgs.msg
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler as qe
import json
import os
import Queue
import threading
import scripts.manipulation as ma

import numpy as np
import math
import ginger_utils

# dmp
#from dmp_module import Dmp

# from dmp_vfc.srv import *
from robots.ginger import Ginger

g_ah = None
g_exit = False

def pack_js_status(code, eventID, actionIndex, message):
    """

    :param code: code;//0:success, 1:fail, 2:....
    :param eventID:
    :param actionIndex:
    :param message:
    :return:
    """
    data = {}
    data['code'] = code
    data['eventID'] = eventID
    data['actionIndex'] = actionIndex
    data['message'] = message
    return json.dumps(data)

class action_handler(object):
    def __init__(self, buffer_size = 0):
        # rospy.init_node('action_handler')
        self.robot = Ginger()
        # messages
        self.action_buffer = Queue.Queue(buffer_size)
        self.status = None
        self.last_rsp = None
        self.last_rsp_sent_cnt = 0
        self.last_rsp_recv_cnt = 0
        self.rsp_timer = None

        self.camera_params = None
        self.rsp_sent = False
        self.action_msg_list = []

        self.action_lock = threading.Lock()
        self.rsp_lock = threading.Lock()
        self.by_service = False
        self.service_rsp = None

        self.message = None
        self.current_event_id = -1
        self.action_list = None
        self.current_action_id = -1
        self.prev_event_id = -1
        self.prev_action_id = -1

        # outputs
        self.br_static = tf2_ros.StaticTransformBroadcaster()
        self.br = tf.TransformBroadcaster()
        self.status_pub = rospy.Publisher('/vfc/status', String, queue_size=1)
        self.execute_pub = rospy.Publisher('/vfc/execute', std_msgs.msg.Bool, queue_size=1)

        rospy.Subscriber('/vfc/camera', String, self.callback_json_tf_camera)
        rospy.Subscriber('/vfc/action', String, self.callback_json_action)
        #self.robot.set_main_body_movement(back_z=-0.5, back_x=None)

    def load_camera_param(self):
        camera_param_path = "/home/nvidia/camera_param.json"
        while True:
            if os.path.exists(camera_param_path):
                with open(camera_param_path) as f:
                    camera_params = f.read()
                    self.camera_params = json.loads(camera_params)
                    rospy.loginfo("camera_params: %s" % self.camera_params)
                    break

            rospy.loginfo("load_camera_param: wait for %s" % camera_param_path)
            rospy.sleep(5)

    def callback_json_tf_camera(self, data):
        sub_json_tf_camera = data.data
        rospy.loginfo("data: %s" % data)

        try:
            tf_camera = json.loads(sub_json_tf_camera)
        except Exception as e:
            #rospy.loginfo("Check json tf message format")
            return False

        #TODO: embed time stamp into TF
        try:
            time_stamp = tf_camera["timestamp"]
            frame = "camera"
            for object in tf_camera["data"]:
                object_name = object["object"]
                pos = object["position"]
                position = [pos["x"], pos["y"], pos["z"]]
                ori = object["orientation"]
                ori_euler = [ori["nx"], ori["ny"], ori["nz"]]
                self.br.sendTransform(position,
                                  qe(ori_euler[0], ori_euler[1], ori_euler[2]),
                                 rospy.Time.now(),
                                  object_name,
                                  frame)

        except Exception as e:
            rospy.loginfo("check json field and values for TF")
            return False
        return True

    def send_rsp(self, rsp):
        self.rsp_lock.acquire()

        try:
            self.last_rsp = rsp
            self.last_rsp_sent_cnt = 1
            self.last_rsp_recv_cnt = 0

            self.status_pub.publish(std_msgs.msg.String(self.last_rsp))

            rospy.loginfo("send_rsp: last_rsp_sent_cnt %s, last_rsp_recv_cnt %s, rsp content: %s" %
                          (self.last_rsp_sent_cnt, self.last_rsp_recv_cnt, rsp))
            rsp_timer = threading.Timer(0.2, self.retransmit_last_rsp)
            rsp_timer.start()
            self.rsp_timer = rsp_timer
        finally:
            self.rsp_lock.release()

    def process_response_ack(self, rsp_ack):
        self.rsp_lock.acquire()

        try:
            rsp_ack_json = json.loads(rsp_ack)
            last_rsp_json = json.loads(self.last_rsp)

            if rsp_ack_json['eventID'] == last_rsp_json['eventID'] and \
                rsp_ack_json['actionIndex'] == last_rsp_json['actionIndex']:
                self.last_rsp_recv_cnt += 1
                rospy.loginfo("callback_json_action: last_rsp_sent_cnt %s, last_rsp_recv_cnt %s" %
                              (self.last_rsp_sent_cnt, self.last_rsp_recv_cnt))
                if self.rsp_timer:
                        rospy.loginfo("process_response_ack: cancel response retransmit timer")
                        self.rsp_timer.cancel()
                        self.rsp_timer = None
            else:
                rospy.logwarn("callback_json_action: not ack for last rsp??")
        finally:
            self.rsp_lock.release()

    def stop_retransmit_last_rsp(self):
        self.rsp_lock.acquire()

        try:
            if self.rsp_timer:
                    rospy.loginfo("stop_retransmit_last_rsp: cancel response retransmit timer")
                    self.rsp_timer.cancel()
                    self.rsp_timer = None
        finally:
            self.rsp_lock.release()

    def retransmit_last_rsp(self):
        self.rsp_lock.acquire()

        try:
            if self.rsp_timer and self.last_rsp:
                self.last_rsp_sent_cnt += 1
                rospy.loginfo("retransmit_last_rsp: last_rsp_sent_cnt %s, last_rsp_recv_cnt %s, rsp content: %s" %
                              (self.last_rsp_sent_cnt, self.last_rsp_recv_cnt, self.last_rsp))
                self.status_pub.publish(std_msgs.msg.String(self.last_rsp))

                if self.last_rsp_recv_cnt == 0 and self.last_rsp_sent_cnt < 100:
                    rsp_timer = threading.Timer(0.2, self.retransmit_last_rsp)
                    rsp_timer.start()
                    self.rsp_timer = rsp_timer
        finally:
            self.rsp_lock.release()

    def callback_json_action(self, data, by_service=False):
        self.action_lock.acquire()

        try:
            self.by_service = by_service
            self.service_rsp = None

            rospy.loginfo("")
            rospy.loginfo("")
            rospy.loginfo("callback_json_action by_service %s, data: %s" % (by_service, data))
            rospy.loginfo("")
            rospy.loginfo("")

            current_msg = json.loads(data.data)

            if "code" in current_msg:
                self.process_response_ack(data.data)
            else:
                if current_msg['cmd'] == 10:
                    self.get_target_pose(current_msg)
                else:
                    if current_msg['cmd'] == 0:
                        current_event_id = current_msg["eventData"]['Id']
                        current_action_id = -1
                    else:
                        current_event_id = current_msg['eventID']  # event_id
                        current_action_id = current_msg['actionIndex']  # action_index

                    status_message = pack_js_status(100, current_event_id, current_action_id,
                                                    "Message Received")
                    self.status_pub.publish(std_msgs.msg.String(status_message))
                    rospy.loginfo("")
                    rospy.loginfo("")
                    rospy.loginfo("callback_json_action: current_event_id %s, current_action_id %s" %
                                  (current_event_id, current_action_id))
                    rospy.loginfo("")
                    rospy.loginfo("")

                    if self.prev_event_id == current_event_id and \
                            self.prev_action_id == current_action_id:
                        rospy.loginfo("callback_json_action: retransmit??")
                    else:
                        self.prev_event_id = current_event_id
                        self.prev_action_id = current_action_id
                        self.stop_retransmit_last_rsp()

                        self.action_msg_list.append(data.data)
        finally:
            self.action_lock.release()

    def handler(self):
        while not rospy.is_shutdown():
            self.robot.pub_joint_state()
            rospy.sleep(0.1)

    def process_action_msg_list(self):
        while not g_exit:
            current_action_msg = None
            self.action_lock.acquire()
            try:
                if self.action_msg_list:
                    current_action_msg = self.action_msg_list.pop(0)
            finally:
                self.action_lock.release()
            if not current_action_msg:
                rospy.sleep(0.05)
                continue

            try:
                self.stop_retransmit_last_rsp()

                rospy.loginfo("")
                rospy.loginfo("")
                rospy.loginfo("process_action_msg_list: current_action_msg %s" % current_action_msg)
                rospy.loginfo("")
                rospy.loginfo("")
                self.message = json.loads(current_action_msg)
                self.motion_controller()
            except Exception as e:
                rospy.loginfo("Check Json action format: %s" % e)

    def motion_controller(self):
        if self.message is not None:
            cmd_type = self.message['cmd']
            rospy.loginfo("cmd_type: %s" % cmd_type)
            if cmd_type == 0:
                # while self.action_list is None:
                success = self.decode_event_data()

                # restart
                #if self.current_event_id == 20:
                self.current_action_id = -1

                self.return_message(success, error_code=20, error_msg="load action list successful")

            else:
                if cmd_type == 1: #exe_action
                    rospy.loginfo("action_list: %s" % self.action_list)
                    if self.action_list is not None:
                        #self.return_message(True, error_code=20, error_msg="load action parameter successful")
                        self.rsp_sent = False
                        success = self.exe_action()
                        if not self.rsp_sent:
                            self.return_message(success)
                    else:
                        rospy.logerr("need to have action list first ...")
                elif cmd_type == 2: # pause
                    pass
                elif cmd_type == 3: # continue
                    pass
                elif cmd_type == 4:  # stop
                    rospy.loginfo("hard stop from cmd of mmo server")
                    #self.return_message(True, error_code=20, error_msg="load stop action parameter successful")
                    status_message = pack_js_status(1, self.current_event_id, self.current_action_id, "hard stopped by MMO server" )
                    self.service_rsp = status_message
                    self.status_pub.publish(std_msgs.msg.String(status_message))
                    exit()
                else:
                    rospy.loginfo("wrong cmd type number")


    def decode_event_data(self):
        self.current_action_id = -1
        self.action_list = None

        event = self.message['eventData']
        self.current_event_id = event['Id']

        while self.action_list is None:
        # if self.action_list is None:
            self.action_list = event['actDatas']
        if self.action_list is not None:
            return True
        else:
            rospy.loginfo("please clear action list first before new event")
            return False

    def exe_action(self):
        self.current_event_id = self.message['eventID']  # event_id
        self.current_action_id = self.message['actionIndex'] # action_index
        self.exe_cmd = self.action_list[self.current_action_id]
        rospy.loginfo("exe_cmd: %s" % self.exe_cmd)
        rospy.loginfo("[exe_action] %d %d" % (self.current_event_id, self.current_action_id))

        if 'ActParameters' in self.exe_cmd:
            parameters = self.exe_cmd['ActParameters']
            if parameters and 'target_type' in parameters:
                rospy.loginfo("target_type %s" % parameters['target_type'])
                target_type = parameters['target_type']
            else:
                rospy.loginfo("target_type not found, assume target type coffee")
                target_type = "coffee"
        else:
            rospy.loginfo("ActParameters not found, assume target type coffee")
            target_type = "coffee"

        ma.set_target_type(target_type)

        if self.current_event_id == 20 and self.current_action_id == 0:
            rospy.loginfo("*******turn around*******")
            self.robot.set_main_body_mode(1)
            self.robot.set_left_hand_mode(1)
            self.robot.set_right_hand_mode(1)
            self.robot.set_left_arm_mode(1)
            self.robot.set_right_arm_mode(1)

            rospy.loginfo("Ginger stand up")
            rospy.loginfo("    initial_preparation")
            ma.initial_preparation(self.robot)
            #rospy.sleep(1)

            self.return_message(True)
            self.rsp_sent = True

            rospy.loginfo("    first_action")
            ma.first_action(self.robot)
            rospy.sleep(1)
            return True

        elif self.current_event_id == 30 and self.current_action_id == 0:
            rospy.loginfo("*******trigger recognition*******")
            return True
        elif self.current_event_id == 40 and self.current_action_id == 0:
            rospy.loginfo("******* recognition failed *******")
            ma.first_action(self.robot, drop_hand=True)
            return True
        elif self.current_event_id == 30 and self.current_action_id == 1:
            rospy.loginfo("*******recognition completed*******")
            return True
        elif self.current_event_id == 50 and self.current_action_id == 0:
            rospy.loginfo("*******parse target pose*******")
            parameters = self.exe_cmd['ActParameters']
            target_pose_ok = self.get_target_pose(parameters)

            if target_pose_ok:
                rospy.loginfo("sendtf")
                grasp_ok = ma.move_to(self.robot)
            else:
                grasp_ok = False

            if not grasp_ok:
                ma.first_action(self.robot, drop_hand=True)

                self.return_message(False)
                self.rsp_sent = True
            else:
                self.return_message(True)
                self.rsp_sent = True

                rospy.loginfo("    turn_back to")
                ma.turn_back(self.robot)
                rospy.loginfo("    last_action to")
                ma.last_action(self.robot)
                rospy.sleep(1)

            return True

        rospy.loginfo("unexpected action")

        return True

    def get_target_pose(self, parameters):
        rospy.loginfo("module type:VFC")

        target_pos = parameters['target_pos']
        target_pos = target_pos['x'], target_pos['y'], target_pos['z']
        rospy.loginfo("")
        rospy.loginfo("")
        rospy.loginfo("target_pos: %s" % str(target_pos))
        rospy.loginfo("")
        rospy.loginfo("")

        target_rot = parameters['target_rot']
        target_rot = target_rot['x'], target_rot['y'], target_rot['z']
        rospy.loginfo("target_rot: %s" % str(target_rot))

        # translation
        translation = target_pos

        # rotation vector to quaternion
        norm_rot = np.linalg.norm(target_rot, ord=2)
        theta = norm_rot
        vec = float(target_pos[0]) / norm_rot, float(target_pos[1]) / norm_rot, float(target_pos[2]) / norm_rot
        vec = float(vec[0]) * math.sin(theta / 2), float(vec[1]) * math.sin(theta / 2), float(vec[2]) * math.sin(
            theta / 2)
        quaternion = [vec[0], vec[1], vec[2], math.cos(theta / 2)]
        quaternion = ginger_utils.normalize_quaternion(quaternion)
        rospy.loginfo("quaternion: %s" % str(quaternion))

        # tf
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "Camera"
        #static_transformStamped.child_frame_id = "Grasp_point"
        static_transformStamped.child_frame_id = "Marker_31"
        static_transformStamped.transform.translation.x = float(translation[0])
        static_transformStamped.transform.translation.y = float(translation[1])
        static_transformStamped.transform.translation.z = float(translation[2])
        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]
        self.br_static.sendTransform(static_transformStamped)

        return ma.setup_marker_pos(self.robot, self.br_static)

    def clear_event(self):
        current_event_id = self.current_event_id
        self.current_event_id = -1
        self.current_action_id = -1
        self.action_list = None
        self.message = None

        # return a message that event was finished and the action list is cleared
        #status_message = pack_js_status(10, current_event_id, None, "event successful, and action list cleared")
        #self.status_pub.publish(std_msgs.msg.String(status_message))


    def return_message(self, success , error_code = 0, error_msg = None):
        if success:
            if self.current_event_id == 20 and self.current_action_id == 0:
                msg = "%f,%f,%f,%f" % (self.camera_params['fx'], self.camera_params['fy'],
                                          self.camera_params['ppx'], self.camera_params['ppy'])
            else:
                msg = "action success"

            status_message = pack_js_status(0, self.current_event_id, self.current_action_id, msg)
        else:
            if error_code == 0:
                error_code = 1
            if not error_msg:
                error_msg = "action failed"
            status_message = pack_js_status(error_code, self.current_event_id, self.current_action_id, error_msg)

        self.service_rsp = status_message
        self.send_rsp(status_message)

        rospy.loginfo("return_message: status_message %s" % status_message)

        if self.current_action_id == len(self.action_list) - 1:
            self.clear_event()
            rospy.loginfo("clear_event\n")


    def execute_action(self):
        if self.commands is not None:
            for index, command in enumerate(self.commands):
                self.action_helpper(command)

    def action_helpper(self, seq, command):
        if seq == int(command['Id'].split('_')[1]):
            act_type = command['ActType']


    def clear_action_buffer(self):
        try:
            while self.action_buffer.not_empty:
                self.action_buffer.get_nowait()
        except Exception as e:
            rospy.loginfo("action buffer not cleared")
            return False
        return True

    def exec_action(self):
        pass


def handle_generic_json(req):
    global g_ah

    rospy.loginfo("handle_generic_json: req %s" % req)
    if req.path == "/ping":
        return req.request_body
    elif req.path == "/vfc/action":
        g_ah.callback_json_action(req.request_body, by_service=True)
        if g_ah.service_rsp:
            return g_ah.service_rsp
        else:
            err_rsp = {
                "err_code": -1,
                "err_msg": "response not set???"
            }
            err_rsp = json.dumps(err_rsp)
            return err_rsp
    else:
        err_rsp = {
            "err_code": -1,
            "err_msg": "unknown path %s" % req.path
        }
        err_rsp = json.dumps(err_rsp)
        return err_rsp


def generic_json_server():
    s = rospy.Service('action_handler/generic_json_server', GenericJSONService, handle_generic_json)
    rospy.loginfo("Ready to handle generic json message")


if __name__ == '__main__':
    g_ah = action_handler()
    g_ah.load_camera_param()
    # generic_json_server()
    t = threading.Thread(target=g_ah.process_action_msg_list)
    t.start()
    g_ah.handler()
    rospy.spin()
    g_exit = True
    rospy.loginfo("Main exit")



