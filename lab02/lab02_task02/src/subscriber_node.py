#!/usr/bin/env python3

import rospy
import random
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped


##TODO: complete your callback function.
def callback(msg, offsets):

    # offsets captures the tuple (offset1, offset2) defined in callback_args

    ##TODO: Add an offset to the incoming x and z coordinates.
    x_new = msg.vector.x + offsets[0]
    z_new = msg.vector.z + offsets[1]

    ##TODO: Print the updated coordinates.
    print('The current point is ')
    print(str(x_new))
    print(str(msg.vector.y))
    print(str(z_new))

def listener():

    ##TODO: Initialize the subscriber node.
    rospy.init_node('subscriber', anonymous = True)
	
    ##TODO: Create two random numbers between 0 and 1.
    offset1 = random.randint(0, 1)
    offset2 = random.randint(0, 1)

    ##TODO: Define the subscriber with the offset arguments
    sub = rospy.Subscriber('chatter', Vector3Stamped, callback, (offset1, offset2))
    
    # callback_args parameter enables to add extra arguments to the callback function
    # >> these arguments are passed as a tuple

    rospy.spin()

if __name__ == '__main__':
    listener()
