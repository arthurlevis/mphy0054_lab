#!/usr/bin/env python3

import numpy as np
from cw2q4.youbotKineBase import YoubotKinematicBase
from cw2q4.youbotKineKDL import YoubotKinematicKDL


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        # Set the offset for theta --> This was updated on 03/12/2024. 
        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]

        # Apply joint offsets to dh parameters
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        # Joint reading polarity signs
        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        frame number. The frame transformation used in the computation are derived from dh parameters and
        joint_readings.
        Args:
            joints_readings (list): the state of the robot joints. In a youbot those are revolute
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint}
                w.r.t the base of the robot.
        """
        assert isinstance(self.dh_params, dict)
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        assert up_to_joint >= 0
        assert up_to_joint <= len(self.dh_params['a'])

        T = np.identity(4)

	# --> This was updated on 23/11/2023. Feel free to use your own code.

        # Apply offset and polarity to joint readings (found in URDF file)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)
            
        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix. Coursework 2 Question 4a.
        Reference - Lecture 5 slide 24.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # Your code starts here ----------------------------

        # For your solution to match the KDL Jacobian, z0 needs to be set [0, 0, -1] instead of [0, 0, 1], since that is how its defined in the URDF.
        # Both are correct.
        
        # Initialise Jacobian
        jacobian = np.zeros((6, 5))
        
        # Base frame z-vector & origin
        z0 = [np.array([0, 0, -1])]
        o0 = [np.zeros(3)]
        
        # Compute FK for each joint
        for i in range(5):
            T = self.forward_kinematics(joint, i+1)
            z0.append(T[0:3, 2])
            o0.append(T[0:3, 3])
        
        # add +1 because range(5) excludes 5, so it misses the last joint
            
        # Compute Jacobian for each joint
        for i in range(5):
            # Linear velocity Jv
            jacobian[0:3, i] = np.cross(z0[i], o0[-1] - o0[i])
            # Angular velocity Jw
            jacobian[3:6, i] = z0[i]
        
        # index -1 in o0 refers to the last element (end effector)
        
        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 2 Question 4c.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        
        # Your code starts here ----------------------------
        
        jacobian = self.get_jacobian(joint)
        JTJ = jacobian.T @ jacobian
        singularity = bool(np.abs(np.linalg.det(JTJ)) < 1e-6)  # select suitable magnitude

        # Your code ends here ------------------------------

        assert isinstance(singularity, bool)
        return singularity


# Check solutions 

base_robot = YoubotKinematicBase()  # contains joint limits
kdl_robot = YoubotKinematicKDL()  # ground truth
my_robot = YoubotKinematicStudent()

max_diff = 0
singularities_found = []

# Iterate over each joint (step = 1 rad)
for j1 in range(int(base_robot.joint_limit_min[0]), int(base_robot.joint_limit_max[0] + 1)):
    for j2 in range(int(base_robot.joint_limit_min[1]), int(base_robot.joint_limit_max[1] + 1)):
        for j3 in range(int(base_robot.joint_limit_min[2]), int(base_robot.joint_limit_max[2] + 1)):
            for j4 in range(int(base_robot.joint_limit_min[3]), int(base_robot.joint_limit_max[3] + 1)):
                for j5 in range(int(base_robot.joint_limit_min[4]), int(base_robot.joint_limit_max[4] + 1)):
                    test_joints = [j1, j2, j3, j4, j5]
                    
                    kdl_jacobian = kdl_robot.get_jacobian(test_joints)
                    my_jacobian = my_robot.get_jacobian(test_joints)
                    
                    max_diff = max(max_diff, np.max(np.abs(kdl_jacobian - my_jacobian)))
                    
                    if my_robot.check_singularity(test_joints):
                        singularities_found.append(test_joints)

print(f"Maximum difference: {max_diff}")
print(f"Singular configurations found: {len(singularities_found)}")