from sensor_msgs.msg import JointState
import rospy

mock = JointState()
mock.name = ["Back_Z",
"Back_X",
"Back_Y",
"Neck_Z",
"Neck_X",
"Head_Y",
"Left_Shoulder_X",
"Left_Shoulder_Y",
"Left_Elbow_Z",
"Left_Elbow_X",
"Left_Wrist_Z",
"Left_Wrist_Y",
"Left_Wrist_X",
"Right_Shoulder_X",
"Right_Shoulder_Y",
"Right_Elbow_Z",
"Right_Elbow_X",
"Right_Wrist_Z",
"Right_Wrist_Y",
"Right_Wrist_X",
"Left_1_1",
"Left_2_1",
"Left_3_1",
"Left_4_1",
"Left_5_1",
"Right_1_1",
"Right_2_1",
"Right_3_1",
"Right_4_1",
"Right_5_1"
]
mock.position = [0]*30



def ginger_tx2():
    pub = rospy.Publisher('/ginger/joint_states', JointState, queue_size=10)
    rospy.init_node('mock_ginger_joint_tx2', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        for name in mock.name:
            mock.position[mock.name.index(name)] = mock.name.index(name)
        pub.publish(mock)
        rate.sleep()


if __name__ == '__main__':
    try:
        ginger_tx2()
    except rospy.ROSInterruptException:
        pass




