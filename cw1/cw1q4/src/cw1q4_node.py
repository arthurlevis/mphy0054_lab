#!/usr/bin/env python3

import rospy
import numpy as np

# TODO: Include all the required service classes
# your code starts here -----------------------------

from cw1q4_srv.srv import quat2zyx, quat2zyxRequest, quat2zyxResponse
from cw1q4_srv.srv import quat2rodrigues, quat2rodriguesRequest, quat2rodriguesResponse

from std_msgs.msg import Float64

# your code ends here -------------------------------


def convert_quat2zyx(request):
    # TODO complete the function
    """Callback ROS service function to convert quaternion to Euler z-y-x representation

    Args:
        request (quat2zyxRequest): cw1q4_srv service message, containing
        the quaternion you need to convert.

    Returns:
        quat2zyxResponse: cw1q4_srv service response, in which 
        you store the requested euler angles 
    """
    assert isinstance(request, quat2zyxRequest)

    # Your code starts here ----------------------------
    
    # Quaternion components
    qw = request.q.w
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z

    # Squared components
    qws = np.power(qw,2)
    qxs = np.power(qx,2)
    qys = np.power(qy,2)
    qzs = np.power(qz,2)

    # Normalize quaternion to represent rotation without scaling
    norm = np.sqrt(qws + qxs + qys + qzs)
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    # Euler angles (in radians)               
    gamma = np.arctan2(2*(qw*qz + qx*qy), (1 - 2 *(qys + qzs))) 
    beta = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1, 1))  
    alpha = np.arctan2(2*(qw*qx + qy*qz), (1 - 2 *(qxs + qys)))

    response = quat2zyxResponse(
        z = Float64(data=gamma), 
        y = Float64(data=beta),    
        x = Float64(data=alpha),  
    )

    # Your code ends here ------------------------------

    assert isinstance(response, quat2zyxResponse)
    return response


def convert_quat2rodrigues(request):
    # TODO complete the function

    """Callback ROS service function to convert quaternion to rodrigues representation
    
    Args:
        request (quat2rodriguesRequest): cw1q4_srv service message, containing
        the quaternion you need to convert

    Returns:
        quat2rodriguesResponse: cw1q4_srv service response, in which 
        you store the requested rodrigues representation 
    """
    assert isinstance(request, quat2rodriguesRequest)

    # Your code starts here ----------------------------

    # Quaternion components
    qw = request.q.w
    qx = request.q.x
    qy = request.q.y
    qz = request.q.z

    # Normalize quaternion to represent rotation without scaling
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

    # Euler-Rodrigues angle
    theta = 2 * np.arccos(qw)

    # Euler-Rodrigues vector
    bx = qx / np.cos(theta/2)
    by = qy / np.cos(theta/2)
    bz = qz / np.cos(theta/2)

    response = quat2rodriguesResponse(
        x = Float64(data=bx),
        y = Float64(data=by),
        z = Float64(data=bz),
    )

    # Your code ends here ------------------------------

    assert isinstance(response, quat2rodriguesResponse)
    return response

def rotation_converter():
    rospy.init_node('rotation_converter')

    #Initialise the services
    rospy.Service('quat2rodrigues', quat2rodrigues, convert_quat2rodrigues)
    rospy.Service('quat2zyx', quat2zyx, convert_quat2zyx)

    rospy.spin()


if __name__ == "__main__":
    rotation_converter()
