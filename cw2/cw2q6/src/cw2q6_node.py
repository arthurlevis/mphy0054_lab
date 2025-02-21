#!/usr/bin/env python

import numpy as np
import rospy
import rosbag
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cw2q4.youbotKineKDL import YoubotKinematicKDL

import PyKDL
from visualization_msgs.msg import Marker


class YoubotTrajectoryPlanning(object):
    def __init__(self):
        # Initialize node
        rospy.init_node('youbot_traj_cw2', anonymous=True)

        # Save question number for check in main run method
        self.kdl_youbot = YoubotKinematicKDL()

        # Create trajectory publisher and a checkpoint publisher to visualize checkpoints
        self.traj_pub = rospy.Publisher('/EffortJointInterface_trajectory_controller/command', JointTrajectory,
                                        queue_size=5)
        self.checkpoint_pub = rospy.Publisher("checkpoint_positions", Marker, queue_size=100)

    def run(self):
        """This function is the main run function of the class. When called, it runs question 6 by calling the q6()
        function to get the trajectory. Then, the message is filled out and published to the /command topic.
        """
        print("run q6a")
        rospy.loginfo("Waiting 5 seconds for everything to load up.")
        rospy.sleep(2.0)
        traj = self.q6()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)

    def q6(self):
        """ This is the main q6 function. Here, other methods are called to create the shortest path required for this
        question. Below, a general step-by-step is given as to how to solve the problem.
        Returns:
            traj (JointTrajectory): A list of JointTrajectory points giving the robot joint positions to achieve in a
            given time period.
        """
        # Steps to solving Q6.

        # Steps to solving Q6.
        # 1. Load in targets from the bagfile (checkpoint data and target joint positions).
        # 2. Compute the shortest path achievable visiting each checkpoint Cartesian position.
        # 3. Determine intermediate checkpoints to achieve a linear path between each checkpoint and have a full list of
        #    checkpoints the robot must achieve. You can publish them to see if they look correct. Look at slides 39 in lecture 7
        # 4. Convert all the checkpoints into joint values using an inverse kinematics solver.
        # 5. Create a JointTrajectory message.

        # Your code starts here ------------------------------

        # 1. 
        target_cart_tf, target_joint_positions = self.load_targets()
        # 2. 
        sorted_order, min_dist = self.get_shortest_path(checkpoints_tf = target_cart_tf)
        # 3.
        full_checkpoint_tfs = self.intermediate_tfs(sorted_order, target_cart_tf, num_points=5)
        # 4. 
        q_checkpoints = self.full_checkpoints_to_joints(
            full_checkpoint_tfs,
            init_joint_position = self.kdl_youbot.kdl_jnt_array_to_list(self.kdl_youbot.current_joint_position)
            )
        # 5.
        traj = JointTrajectory()

        self.publish_traj_tfs(full_checkpoint_tfs)
        elapsed_time = 20  # total time
        interval = 1  # time between each movement

        for i in range(q_checkpoints.shape[1]):
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = q_checkpoints[:, i]
            elapsed_time += interval
            trajectory_point.time_from_start.secs = elapsed_time
            traj.points.append(trajectory_point)

        # Your code ends here ------------------------------

        assert isinstance(traj, JointTrajectory)
        return traj

    def load_targets(self):
        """This function loads the checkpoint data from the 'data.bag' file. In the bag file, you will find messages
        relating to the target joint positions. You need to use forward kinematics to get the goal end-effector position.
        Returns:
            target_cart_tf (4x4x5 np.ndarray): The target 4x4 homogenous transformations of the checkpoints found in the
            bag file. There are a total of 5 transforms (4 checkpoints + 1 initial starting cartesian position).
            target_joint_positions (5x5 np.ndarray): The target joint values for the 4 checkpoints + 1 initial starting
            position.
        """
        # Defining ros package path
        rospack = rospkg.RosPack()
        path = rospack.get_path('cw2q6')

        # Initialize arrays for checkpoint transformations and joint positions
        target_joint_positions = np.zeros((5, 5))
        # Create a 4x4 transformation matrix, then stack 6 of these matrices together for each checkpoint
        target_cart_tf = np.repeat(np.identity(4), 5, axis=1).reshape((4, 4, 5))

        # Load path for selected question
        bag = rosbag.Bag(path + '/bags/data.bag')
        # Get the current starting position of the robot
        target_joint_positions[:, 0] = self.kdl_youbot.kdl_jnt_array_to_list(self.kdl_youbot.current_joint_position)
        # Initialize the first checkpoint as the current end effector position
        target_cart_tf[:, :, 0] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, 0])

        # Your code starts here ------------------------------

        # Load target joint positions & convert them into Cartesian transforms using FK 
        for i, (_, msg, _) in enumerate(bag.read_messages(topics=["joint_data"])):

            target_joint_positions[:, i+1] = msg.position
            target_cart_tf[:, :, i+1] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, i+1])
        
        # Initial end-effector positions 
        initial_end_effector_position = target_cart_tf[:3, -1, 0]
        print(f"Initial end-effector position: {initial_end_effector_position}")

        # Target end-effector positions 
        ch1_end_effector_position = target_cart_tf[:3, -1, 1]
        print(f"Checkpoint 1 end-effector position: {ch1_end_effector_position}")

        ch2_end_effector_position = target_cart_tf[:3, -1, 2]
        print(f"Checkpoint 2 end-effector position: {ch2_end_effector_position}")

        ch3_end_effector_position = target_cart_tf[:3, -1, 3]
        print(f"Checkpoint 3 end-effector position: {ch3_end_effector_position}")

        ch4_end_effector_position = target_cart_tf[:3, -1, 4]
        print(f"Checkpoint 4 end-effector position: {ch4_end_effector_position}")

        # Your code ends here ------------------------------

        # Close the bag
        bag.close()

        assert isinstance(target_cart_tf, np.ndarray)
        assert target_cart_tf.shape == (4, 4, 5)
        assert isinstance(target_joint_positions, np.ndarray)
        assert target_joint_positions.shape == (5, 5)

        return target_cart_tf, target_joint_positions

    def get_shortest_path(self, checkpoints_tf):
        """This function takes the checkpoint transformations and computes the order of checkpoints that results
        in the shortest overall path.
        Args:
            checkpoints_tf (np.ndarray): The target checkpoints transformations as a 4x4x5 numpy ndarray.
        Returns:
            sorted_order (np.array): An array of size 5 indicating the order of checkpoint
            min_dist:  (float): The associated distance to the sorted order giving the total estimate for travel
            distance.
        """

        # Your code starts here ------------------------------

        # Get xyz coordinates from transforms
        xyz = checkpoints_tf[0:3, -1, :]  
        
        # Calculate distance matrix
        distance_matrix = np.zeros((5, 5))
        for i in range(5):
            for j in range(5):
                distance_matrix[i, j] = np.linalg.norm(xyz[:, i] - xyz[:, j])
                
        # Only permute points 1-4 (initial point 0 must be fixed)
        min_dist = float('inf')
        best_path = None
        remaining_points = list(range(1, 5))
        
        from itertools import permutations
        for perm in permutations(remaining_points):
            path = [0] + list(perm)  # fixed initial point
            dist = sum(distance_matrix[path[i]][path[i+1]] for i in range(4))
            if dist < min_dist:
                min_dist = dist
                best_path = path
                
        sorted_order = np.array(best_path)

        print(f"Shortest path: {sorted_order}")
        print(f"Path distance: {min_dist}")

        # Your code ends here ------------------------------

        assert isinstance(sorted_order, np.ndarray)
        assert sorted_order.shape == (5,)
        assert isinstance(min_dist, float)

        return sorted_order, min_dist

    def publish_traj_tfs(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.b = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0, -1, i]
            marker.pose.position.y = tfs[1, -1, i]
            marker.pose.position.z = tfs[2, -1, i]
            self.checkpoint_pub.publish(marker)

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """This function takes the target checkpoint transforms and the desired order based on the shortest path sorting, 
        and calls the decoupled_rot_and_trans() function.
        Args:
            sorted_checkpoint_idx (list): List describing order of checkpoints to follow.
            target_checkpoint_tfs (np.ndarray): the state of the robot joints. In a youbot those are revolute
            num_points (int): Number of intermediate points between checkpoints.
        Returns:
            full_checkpoint_tfs: 4x4x(4xnum_points + 5) homogeneous transformations matrices describing the full desired
            poses of the end-effector position.
        """

        # Your code starts here ------------------------------

        # Initialize output
        full_checkpoint_tfs = np.zeros((4, 4, 4*num_points + 5))

        # Loop through the sorted checkpoints except the last one
        for i in range(sorted_checkpoint_idx.size - 1):
            full_checkpoint_tfs[:, :, i*(num_points+1)] = target_checkpoint_tfs[:, :, sorted_checkpoint_idx[i]]
            
            full_checkpoint_tfs[:, :, i*num_points+1+i:(i+1)*num_points+1+i] = self.decoupled_rot_and_trans(
                target_checkpoint_tfs[:, :, sorted_checkpoint_idx[i]],
                target_checkpoint_tfs[:, :, sorted_checkpoint_idx[i+1]], 
                num_points
            )
        
        # Add final checkpoint
        full_checkpoint_tfs[:, :, -1] = target_checkpoint_tfs[:, :, sorted_checkpoint_idx[-1]]
        
        # Your code ends here ------------------------------
       
        return full_checkpoint_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """This function takes two checkpoint transforms and computes the intermediate transformations
        that follow a straight line path by decoupling rotation and translation.
        Args:
            checkpoint_a_tf (np.ndarray): 4x4 transformation describing pose of checkpoint a.
            checkpoint_b_tf (np.ndarray): 4x4 transformation describing pose of checkpoint b.
            num_points (int): Number of intermediate points between checkpoint a and checkpoint b.
        Returns:
            tfs: 4x4x(num_points) homogeneous transformations matrices describing the full desired
            poses of the end-effector position from checkpoint a to checkpoint b following a linear path.
        """

        # Your code starts here ------------------------------

        # Time step between 0 & 1
        delta_time = 1/(num_points+1)

        # Extract translation & rotation matrices from checkpoints
        start_position = checkpoint_a_tf[:3, -1]
        end_position = checkpoint_b_tf[:3, -1]

        start_rotation = checkpoint_a_tf[:3, :3]
        end_rotation = checkpoint_b_tf[:3, :3]

        #  Initialize output
        tfs = np.zeros([4, 4, num_points])

        # Decouple translation & rotation
        for step in range(num_points):

            t = (step+1)*delta_time

            # Interpolate position
            tfs[0:3, -1, step] = start_position + t * (end_position - start_position)
           
            # Interpolate rotation
            from scipy.linalg import logm, expm  # use scipy instead of numpy for numerical stability
            tfs[:3, :3, step] = start_rotation @ expm(logm(start_rotation.T @ end_rotation) * t)
           
            tfs[3, :3, step] = 0
            tfs[3, 3, step] = 1

        # Your code ends here ------------------------------

        return tfs

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        """This function takes the full set of checkpoint transformations, including intermediate checkpoints, 
        and computes the associated joint positions by calling the ik_position_only() function.
        Args:
            full_checkpoint_tfs (np.ndarray, 4x4xn): 4x4xn transformations describing all the desired poses of the end-effector
            to follow the desired path.
            init_joint_position (np.ndarray):A 5x1 array for the initial joint position of the robot.
        Returns:
            q_checkpoints (np.ndarray, 5xn): For each pose, the solution of the position IK to get the joint position
            for that pose.
        """
        
        # Your code starts here ------------------------------

        # Initialize
        q_checkpoints = np.zeros([5, full_checkpoint_tfs.shape[2]])

        # Calculate the joint positions of the first checkpoint using initial joint positions
        q_checkpoints[:, 0], _ = self.ik_position_only(
            full_checkpoint_tfs[:, :, 0], init_joint_position
            )

        # Calculate the remaining joint positions
        for i in range(1, full_checkpoint_tfs.shape[2]):
            
            q_checkpoints[:, i], _ = self.ik_position_only(
                full_checkpoint_tfs[:, :, i], q_checkpoints[:, i-1]
                )

        # Your code ends here ------------------------------

        return q_checkpoints

    def ik_position_only(self, pose, q0):
        """This function implements position only inverse kinematics.
        Args:
            pose (np.ndarray, 4x4): 4x4 transformations describing the pose of the end-effector position.
            q0 (np.ndarray, 5x1):A 5x1 array for the initial starting point of the algorithm.
        Returns:
            q (np.ndarray, 5x1): The IK solution for the given pose.
            error (float): The Cartesian error of the solution.
        """
        # Some useful notes:
        # We are only interested in position control - take only the position part of the pose as well as elements of the
        # Jacobian that will affect the position of the error.

        # Your code starts here ------------------------------

        # Set hyperparameters
        alpha = 0.5  # learning rate
        max_iter = 200  # maximum iterations
        tolerance = 0.01  # error tolerance

        # Initialize error to a large value
        error = 1

        # Initialise joint angles
        q = list(np.array(q0).ravel())
        
        # Target end-effector position
        target_end_effector_position = pose[:3, -1]

        # IK
        for i in range(max_iter):
        
            if error < tolerance:
                break
           
            # Calculate the current position of the end-effector using FK
            x = np.array(self.kdl_youbot.forward_kinematics(q))[0:3, -1]
           
            # Calculate the Jacobian matrix
            Jacobian = self.kdl_youbot.get_jacobian(q)[0:3, :]
           
            # Calculate the error between target position (xe) & current position (x)
            error = np.linalg.norm(target_end_effector_position-x)
           
            # Update the joint positions based on the Jacobian & error
            q = q + alpha * Jacobian.T.dot(target_end_effector_position - x)
           
            # Convert the updated joint positions back to a list
            q = list(np.array(q).ravel())

        # Convert the final joint positions to a real number array
        q = np.array(q).real
        
        # Your code ends here ------------------------------

        return q, error

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        """This converts a list to a KDL jnt array.
        Args:
            joints (joints): A list of the joint values.
        Returns:
            kdl_array (PyKDL.JntArray): JntArray object describing the joint position of the robot.
        """
        kdl_array = PyKDL.JntArray(5)
        for i in range(0, 5):
            kdl_array[i] = joints[i]
        return kdl_array


if __name__ == '__main__':
    try:
        youbot_planner = YoubotTrajectoryPlanning()
        youbot_planner.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
