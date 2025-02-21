#!/usr/bin/env python3

import rospy
import numpy as np

from lab03_example_srv.srv import point_rotResponse
from lab03_example_srv.srv import point_rot

def handle_point_rotation(req):

    px = req.p.x
    py = req.p.y
    pz = req.p.z

    qx = req.q.x
    qy = req.q.y
    qz = req.q.z
    qw = req.q.w

    # service mechanism: request instance automatically passed on to function from client
    # through a service called 'rotate_pt' --> no need to create it in the server node

    qxs = np.power(qx,2)
    qys = np.power(qy,2)
    qzs = np.power(qz,2)

    res = point_rotResponse()

    # Quaternion to rotation matrix
    res.out_p.x = px * (1 - 2*qys - 2*qzs) + py * (2*(qx*qy - qz*qw)) + pz*  (2*(qx*qz - qy*qw))
    res.out_p.y = px * (2*(qx*qy + qz*qw)) + py * (1 - 2*qxs - 2*qzs) + pz * (2*(qy*qz - qx*qw))
    res.out_p.z = px * (2*(qx*qz - qy*qw)) + py * (2*(qy*qz + qx*qw)) + pz * (1 - 2*qys - 2*qxs)

    return res

def rortate_point_service():
    rospy.init_node('rotate_point', anonymous=True)
    s = rospy.Service('rotate_pt', point_rot, handle_point_rotation)
    rospy.spin()  # maintains service node active to handle requests

if __name__ == '__main__':
    rortate_point_service()