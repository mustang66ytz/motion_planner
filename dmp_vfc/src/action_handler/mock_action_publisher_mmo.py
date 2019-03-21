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
        # data = {"cmd":0,"eventData":{"Id":12,"actDatas":[{"Id":"act_0","ActType":1,"ActParameters":{"Time":1.5,"handType":0,"target_pos":{"x":10.0,"y":2.0,"z":6.0},"target_rot":{"x":0.1,"y":0.0,"z":0.5}},"homeType":1},{"Id":"act_1","ActType":0,"ActParameters":{"Time":1.5,"handType":0,"target_pos":{"x":10.0,"y":2.0,"z":6.0},"target_rot":{"x":0.1,"y":0.0,"z":0.5}},"homeType":1},{"Id":"act_2","ActType":1,"ActParameters":{"Time":1.5,"handType":0,"target_pos":{"x":10.0,"y":2.0,"z":6.0},"target_rot":{"x":0.1,"y":0.0,"z":0.5}},"homeType":1}]},"eventID":12,"actionIndex":-1}
        data = {"cmd":0,"eventData": {"Id":12,"actDatas":[ {"Id":"act_0","ActType":"DMP","Time":1.5,"ActParameters":{"handType":"left","target_pos":{"x":10.0,"y":2.0,"z":6.0},"target_rot":{"x":0.1,"y":0.0,"z":0.5}},"ActStateParameters":"null","homeType":1}, {"Id":"act_1","ActType":"VFC","Time":1.5,"ActParameters":{"handType":"left","target_pos":{"x":10.0,"y":2.0,"z":6.0},"target_rot":{"x":0.1,"y":0.0,"z":0.5}},"ActStateParameters":"null","homeType":1}, {"Id":"act_2","ActType":"State","Time":1.5,"ActParameters":"null","ActStateParameters":{"move_group":"left_hand","mode":1,"valueList":[{"value":25.0},{"value":25.0},{"value":25.0},{"value":25.0},{"value":25.0}]},"homeType":1}]}, "eventID":12, "actionIndex":-1}




        # data["actionName"] = "manipulateObject"
        # data["seq"] = seq
        # data["object"] = "latte_btn"
        # data["handSelection"] = "left"
        # data["manipulation"] = "grasp"
        # data["start"]={"position":{"x":100,"y":100,"z":100},"orientation":{"nx":0,"ny":0,"nz":0}}
        # data["target"]={"position":{"x":100,"y":100,"z":100},"orientation":{"nx":0,"ny":0,"nz":0}}
        #             # '"startCondition":{"handstate": "open"},' \
        #             # '"finishCondition":{"handstate": "open"},' \
        # data["modeSelection"] = "DMP"
        # data["timeout"]=1000
                    # '}'
        action = js.dumps(data)
        pub.publish(action)


        data = {"cmd":1,"eventData":"null","eventID":1,"actionIndex":2}
        action = js.dumps(data)
        pub.publish(action)
        print(data)
        rate.sleep()
        # seq += 1


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
