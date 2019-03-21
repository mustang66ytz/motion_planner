import rospy
from std_msgs.msg import String
import std_msgs.msg
import tf
from robots.ginger import Ginger
import visual_servoing.manipulation as vsm


robot = Ginger()
robot.rate.sleep()

# Set the control mode
robot.set_main_body_mode(1)
robot.set_left_hand_mode(1)
robot.set_right_hand_mode(1)
robot.set_left_arm_mode(1)
robot.set_right_arm_mode(1)

robot.rospy.loginfo("Ginger stand up")
# robot.go_zero()
# robot.set_head_movement(0, 0.75)

target_id = 5
# while True:

target_pose = vsm.get_target(robot, target_id)

def callback(data):
    print(data.data)
    if data.data:
        robot.rospy.sleep(1)
        robot.set_left_arm_joint([0, 0, 0, 0, 0, 0, 0], interpolation=1)
        robot.rospy.sleep(1)
        vsm.move_to(robot, target_pose)

rospy.Subscriber('/vfc/execute', std_msgs.msg.Bool, callback)
rospy.spin()


