#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('/vfc/camera', String, queue_size=10)
    rospy.init_node('mock_tf_camera_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        tf_camera = '{"actionName": "notify3DObjects",' \
                    ' "seq": 1, ' \
                    '"timestamp":11111, ' \
                    '"frame":"", ' \
                    '"objects":[{"object": "toy_1", "position":{"x":100,"y":100,"z":100}, "orientation":{"nx":0, "ny":0, "nz":0}}, ' \
                    '           {"object": "toy_2", "position":{"x":100,"y":10,"z":100}, "orientation":{"nx":0, "ny":0, "nz":0}}]}'
        pub.publish(tf_camera)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

