#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

import rospy
import rospkg
import rosbag
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL
    

class Iiwa14ForwardDynamics():

    def trajectory(self):
        
        # Load bag file
        rospack = rospkg.RosPack()
        path = rospack.get_path('cw3q5') + '/bag/cw3q5.bag'
        bag = rosbag.Bag(path)

        # Get trajectory message
        for _, msg, _ in bag.read_messages(topics=['/iiwa/EffortJointInterface_trajectory_controller/command']):
            msg.header.stamp = rospy.Time.now()  # update timestamp
            bag.close()
            return msg  # JointTrajectory()

        bag.close()
        return None
    

    def joint_accelerations(self, joint_state):

        # Check if a counter doesn't exist 
        if not hasattr(self, 'plot_counter'):
            self.plot_counter = 0
        
        # Increment for every new joint state 
        self.plot_counter += 1
        
        # Plot every 5th point
        if self.plot_counter % 5 == 0:

            # Joint states data
            q = np.array(joint_state.position)
            q_dot = np.array(joint_state.velocity)
            tau = np.array(joint_state.effort)

            # Dynamic components
            B = Iiwa14DynamicKDL.get_B(Iiwa14DynamicKDL(), q)
            C_qdot = Iiwa14DynamicKDL.get_C_times_qdot(Iiwa14DynamicKDL(), q, q_dot)
            g = Iiwa14DynamicKDL.get_G(Iiwa14DynamicKDL(), q)

            # Lecture 9 slide 25
            a = (np.linalg.inv(B) @ (tau - C_qdot - g))  #1D array compatible with plt.plot()

            # Define time-axis
            stamp = joint_state.header.stamp
            time = stamp.secs + stamp.nsecs *1e-9

            # Plot joint accelerations
            plt.plot(time, a[:,0], 'k.', markersize=4) 
            plt.plot(time, a[:,1], 'b.', markersize=4)
            plt.plot(time, a[:,2], 'g.', markersize=4)
            plt.plot(time, a[:,3], 'r.', markersize=4)
            plt.plot(time, a[:,4], 'c.', markersize=4)
            plt.plot(time, a[:,5], 'm.', markersize=4)
            plt.plot(time, a[:,6], 'y.', markersize=4)

            # Label joint accelerations
            colors = ['k', 'b', 'g', 'r', 'c', 'm', 'y']
            labels = [plt.Line2D([0],[0], marker='.', c=c, label=f'joint {i+1}', linestyle='none') 
                    for i, c in enumerate(colors)]
            plt.legend(handles=labels)

            # Plot
            plt.xlabel('Time (s)')
            plt.ylabel('Acceleration (rad/s2)')
            plt.draw()
            plt.pause(1e-4)  # delay to display real-time data 

        return a


if __name__ == '__main__':
    
    try:        

        # Initialize node
        rospy.init_node('cw3', anonymous=True)
         
        iiwa14_fd = Iiwa14ForwardDynamics()
    
        # Initialize publisher
        trajectory_pub = rospy.Publisher('/iiwa/EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size=5)

        # Define trajectory
        joint_traj  = iiwa14_fd.trajectory()

        # Publish trajectory with artificial delay
        rospy.sleep(1)

        trajectory_pub.publish(joint_traj)

        rospy.sleep(1)

        # Subscribe to joint states topic & compute joint accelerations
        joint_state_sub = rospy.Subscriber('/iiwa/joint_states', JointState, iiwa14_fd.joint_accelerations)

        # Maintain plot open
        plt.ion()
        plt.show()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass