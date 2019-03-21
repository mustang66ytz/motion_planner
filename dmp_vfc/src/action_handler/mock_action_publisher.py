#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json as js

def talker():
    pub = rospy.Publisher('/vfc/action', String, queue_size=1)
    rospy.init_node('mock_action_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    seq = 0
    while not rospy.is_shutdown():
        data = {}
        data["actionName"] = "manipulateObject"
        data["seq"] = seq
        data["object"] = "latte_btn"
        data["handSelection"] = "left"
        data["manipulation"] = "grasp"
        data["start"]={"position":{"x":100,"y":100,"z":100},"orientation":{"nx":0,"ny":0,"nz":0}}
        data["target"]={"position":{"x":100,"y":100,"z":100},"orientation":{"nx":0,"ny":0,"nz":0}}
                    # '"startCondition":{"handstate": "open"},' \
                    # '"finishCondition":{"handstate": "open"},' \
        data["modeSelection"] = "DMP"
        data["timeout"]=1000
                    # '}'
        action = js.dumps(data)
        pub.publish(action)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
