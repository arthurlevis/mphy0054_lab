#!/usr/bin/env python3

import rospy
import random
from lab03_example_msg.msg import test_msg

def rot_publisher():
    
    rospy.init_node('rotation_publisher',anonymous = True)
    pub = rospy.Publisher('publish_rotation', test_msg, queue_size=100)
    rate = rospy.Rate(10)  # 10Hz

    rot_msg = test_msg()

    while not rospy.is_shutdown():

        rot_msg.rotx.data = random.uniform(-2.0, 2.0)
        rot_msg.roty.data = random.uniform(-1.0, 1.0)
        rot_msg.rotz.data = random.uniform(-2.1, 1.8)

        pub.publish(rot_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rot_publisher()
    except rospy.ROSInterruptException:
        pass
