
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class RobotInterface():

    def __init__(self, robot, arm='left'):
        self.robot = robot
        self.arm = arm
        self.joint_state = JointState()
        self.set_arm(arm)

    def is_shutdown(self):
        return self.robot.rospy.is_shutdown()

    def sleep(self):
        self.robot.rate.sleep()

    def get_arm(self, arm):
        return self.arm

    def set_arm(self, arm):
        self.arm = arm

        if self.arm == 'left':
            self.joint_state.name = [
                'Left_Shoulder_X', 'Left_Shoulder_Y',
                'Left_Elbow_Z', 'Left_Elbow_X',
                'Left_Wrist_Z', 'Left_Wrist_Y', 'Left_Wrist_X']
        elif self.arm == 'right':
            self.joint_state.name = [
                'Right_Shoulder_X', 'Right_Shoulder_Y',
                'Right_Elbow_Z', 'Right_Elbow_X',
                'Right_Wrist_Z', 'Right_Wrist_Y', 'Right_Wrist_X']
        else:
            return

    def goto_pose(self, pose):
        position = (pose.position.x, pose.position.y, pose.position.z)
        orientation = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)

        if self.arm == 'left':
           self.robot.set_left_arm_EEF(position, orientation)
        elif self.arm == 'right':
           self.robot.set_righst_arm_EEF(position, orientation)
        else:
           return
        pass

    def goto_state(self, joint_state):
        position = []
        for name in self.joint_state.name:
            if name in joint_state.name:
                position.append(
                    joint_state.position[joint_state.name.index(name)])
            else:
                return

        if self.arm == 'left':
            self.robot.set_left_arm_joint(position, interpolarion=1)
        elif self.arm == 'right':
            self.robot.set_right_arm_joint(position, interpolarion=1)
        else:
            return

    def get_pose(self):
        if self.arm == 'left':
           position, orientation = self.robot.get_left_arm_EEF()
        elif self.arm == 'right':
           position, orientation = self.robot.get_right_arm_EEF()
        else:
           return

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        return pose
        # pass

    def get_state(self):
        if self.arm == 'left':
            self.joint_state.position = self.robot.get_left_arm_joint()
        elif self.arm == 'right':
            self.joint_state.position = self.robot.get_right_arm_joint()
        else:
            return
        return self.joint_state



