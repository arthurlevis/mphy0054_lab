#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float64


def joint_pub():
    rospy.init_node('trajectory_generator', anonymous=True)
    rate = rospy.Rate(10)

    ##TODO: Define each joint publisher, by identifying the appropriate publishing topic.
    pub1 = rospy.Publisher('Joint_J1_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('Joint_J2_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('Joint_J3_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('Joint_J4_controller/command', Float64, queue_size=100)

    ##TODO: Define the messages to be published.
    joint1_msg = Float64()
    joint2_msg = Float64()
    joint3_msg = Float64()
    joint4_msg = Float64()


    while not rospy.is_shutdown():

        t = rospy.Time.now().secs
	
        ##TODO: Define the joint trajectories.
        joint1_msg.data = (200 * math.pi / 180) * math.sin(2 * math.pi * t / 10)
        joint2_msg.data = (50 * math.pi / 180) * math.sin(2 * math.pi * t / 12)
        joint3_msg.data = (-80 * math.pi / 180) * math.sin(2 * math.pi * t / 15)
        joint4_msg.data = (60 * math.pi / 180) * math.sin(2 * math.pi * t / 11)

	    ##TODO: Publish all messages.
        pub1.publish(joint1_msg)
        pub2.publish(joint2_msg)
        pub3.publish(joint3_msg)
        pub4.publish(joint4_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
