import rospy
from std_msgs.msg import String
import std_msgs.msg
import tf
from robots.ginger import Ginger


ginger = Ginger()
def callback(data):
    print(data.data)
    if data.data:
        ginger.set_left_arm_joint([-1, 0, 0, 0, 0,0,0], interpolation=1)
    else:
        ginger.set_left_arm_joint([0, 0, 0, 0, 0,0,0], interpolation=1)

rospy.Subscriber('/vfc/execute', std_msgs.msg.Bool, callback)
rospy.spin()
