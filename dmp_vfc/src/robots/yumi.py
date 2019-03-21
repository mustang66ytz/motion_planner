from yumipy import YuMiRobot
from baserobot import Baserobot
from utils.constants import YumiConstants as yc

class Yumi(Baserobot):
    def __init__(self):
        self.robot = YuMiRobot()

    def get_robot(self):
        return self.robot


    def get_hand(self, hand):
        assert hand == 'right' or 'left'
        if hand == 'right':
            return self.robot.right
        else:
            return self.robot.left

    def set_speed(self, hand, speed=100):
        assert 0 <= speed <= 500, "Speed is out of 0 ~ 500 range"
        assert hand == 'left' or 'right' or 'all', "Must define a hand"
        if hand == 'all':
            self.get_hand('right').set_speed(self.robot.get_v(speed))
            self.get_hand('left').set_speed(self.robot.get_v(speed))
        else:
            self.get_hand(hand).set_speed(self.robot.get_v(speed))

    def get_pose(self, hand):
            """
            :param hand: 'right' (right_arm) or 'left' (left_arm)
            :return: end-effector pose in 4 by 4 matrix
            """
            return self.get_hand(hand).get_pose(raw_res=False)

    def get_state(self, hand):
            return self.get_hand(hand).get_state(raw_res=False)

    def go_center(self, hand):
        return self.get_hand(hand).goto_pose(yc.board_center, relative=True)

    def go_pickup_home(self, hand):
        if hand == 'right':
            self.get_hand(hand).goto_state(yc.right_pickup_home)
        elif hand == 'left':
            self.get_hand(hand).goto_state(yc.left_pickup_home)
        # return self.get_hand(hand).goto_pose(T_rightH_yumi.as_frames('yumi', 'world'), relative=True)

    def go_threading_home(self, hand):
        if hand == 'right':
            self.get_hand('right').goto_state(yc.right_threading_home)
            self.move_delta('right', trans=[-0.08, 0.006, 0], rotation=None)
        elif hand == 'left':
            self.get_hand('left').goto_state(yc.left_threading_home)
            self.move_delta('left', trans=[-0.035, -0.04, 0.004], rotation=None)

    def move_delta(self, hand, trans, rotation=None):
            self.get_hand(hand).goto_pose_delta(trans, rotation)
