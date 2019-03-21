from yumi import Yumi
from dummyrobot import Dummyrobot

class RobotFactory():
    def create_robot(typ):
        if typ == "yumi":
         return Yumi()
        if typ == "blah":
         return Dummyrobot()
         assert 0, "Bad robot creation: " + type

    factory = staticmethod(create_robot)
