#!/usr/bin/env python3

import rospy
import numpy as np
import random
import time

from lab03_example_srv.srv import point_rotRequest
from lab03_example_srv.srv import point_rot

def point_rotation_client():
    
    rospy.wait_for_service('rotate_pt')
    
    # no need to initialize a new ROS node because client is only connecting to a server
    # (no additional id or logging)    
    
    while not rospy.is_shutdown():
        
        client = rospy.ServiceProxy('rotate_pt', point_rot)  # communication with server
        req = point_rotRequest()

        req.p.x = random.uniform(-2.0, 2.0)
        req.p.y = random.uniform(-2.1, 2.3)
        req.p.z = random.uniform(-1.0, 1.0)

        quaternion = np.random.rand(4)  # array of four numbers between 0 & 1
        quaternion = quaternion / np.linalg.norm(quaternion)  # normalises the quaternion

        req.q.x = quaternion[0]
        req.q.y = quaternion[1]
        req.q.z = quaternion[2]
        req.q.w = quaternion[3]

        res = client(req)  # initialises request & returns response

        print(res)

        time.sleep(3)  # 3 seconds before next request

if __name__ == '__main__':
    try:
        point_rotation_client()
    except rospy.ROSInterruptException:
        pass
