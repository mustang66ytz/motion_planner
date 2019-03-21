from baserobot import Baserobot
from dummyrobotArm import *


class Dummyrobot(Baserobot):
    def __init__(self):
        self.robot = None
        self.robotRightHand = None
        self.robotLeftHand = None
        # self.robotState = None

    def get_robot(self):
        return None

    def get_hand(self, hand):
        assert hand == 'right' or 'left'
        if hand == 'right':
            self.robotRightHand = DummyrobotArm('RightArm')
            return self.robotRightHand
        else:
            self.robotLeftHand = DummyrobotArm('LeftArm')
            return self.robotLeftHand
        return None
    #
    # def set_speed(self, hand, speed=100):
    #     pass

    # def get_pose(self, hand):
    #     return self.get_hand(hand).get

    # def set_state(self, hand, vals):
    #         return self.get_hand(hand).set_state(vals)

    def get_state(self, hand):
        return self.get_hand(hand).get_state()

    # def goto_pose(self, hand):
    #     return None
    #
    # def goto_state(self, hand):
    #     return None
    #
    # def go_center(self, hand):
    #     pass
    #
    # def go_pickup_home(self, hand):
    #     pass
    #
    # def go_threading_home(self, hand):
    #     pass
    #
    # def move_delta(self, hand, trans, rotation=None):
    #     pass
