#!/usr/bin/env python3

import numpy as np
from cw3q2.iiwa14DynBase import Iiwa14DynamicBase
from cw3q2.iiwa14DynKDL import Iiwa14DynamicKDL


class Iiwa14DynamicRef(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicRef, self).__init__(tf_suffix='ref')

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """This function solve forward kinematics by multiplying frame transformation up until a specified
        joint. Reference Lecture 9 slide 13.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematics.
                Defaults to 7.
        Returns:
            np.ndarray The output is a numpy 4*4 matrix describing the transformation from the 'iiwa_link_0' frame to
            the selected joint frame.
        """

        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)

        T = np.identity(4)
        # iiwa base offset
        T[2, 3] = 0.1575

        # 1. Recall the order from lectures. T_rot_z * T_trans * T_rot_x * T_rot_y. You are given the location of each
        # joint with translation_vec, X_alpha, Y_alpha, Z_alpha. Also available are function T_rotationX, T_rotation_Y,
        # T_rotation_Z, T_translation for rotation and translation matrices.
        # 2. Use a for loop to compute the final transformation.
        for i in range(0, up_to_joint):
            T = T.dot(self.T_rotationZ(joints_readings[i]))
            T = T.dot(self.T_translation(self.translation_vec[i, :]))
            T = T.dot(self.T_rotationX(self.X_alpha[i]))
            T = T.dot(self.T_rotationY(self.Y_alpha[i]))

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"

        return T

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Given the joint values of the robot, compute the Jacobian matrix at the centre of mass of the link.
        Reference - Lecture 9 slide 14.

        Args:
            joint_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute the Jacobian.
            Defaults to 7.

        Returns:
            jacobian (numpy.ndarray): The output is a numpy 6*7 matrix describing the Jacobian matrix defining at the
            centre of mass of a link.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ----------------------------

        # Initialize jacobian & list of transforms
        jacobian = np.zeros((6,7))
        T_list = []

        # COM position vector of link i
        Tcm_i = self.forward_kinematics_centre_of_mass(joint_readings, up_to_joint)
        pl_i   = Tcm_i[0:3,3]

        for j in range(up_to_joint):
            
            T_list.append(self.forward_kinematics(joint_readings, j))
            T = T_list[j]  # homegenous transform
            z = T[:3,2]    # rotation axis
            p = T[:3,3]    # joint frame position vector

            # Linear & angular components (lecture 9 slide 14)
            jacobian[:3, j] = np.cross(z, (pl_i - p))
            jacobian[3:, j] = z       

        # Your code ends here ------------------------------

        assert jacobian.shape == (6, 7)
        return jacobian

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        B = np.zeros((7, 7))
        
	    # Your code starts here ------------------------------

        for i in range(1,8): 

            # Mass
            m = self.mass[i-1]
            d = self.link_cm [i-1]

            # Jacobian at CoM
            jacobian = self.get_jacobian_centre_of_mass(joint_readings, i)
            Jp = jacobian[:3, :]
            Jo = jacobian[3:, :]

            # Inertia tensor about frame origin
            # self.Ixyz contains principal moments of inertia (urdf: cross products = 0)
            Ig = np.diag(self.Ixyz[i-1])
            Io = Ig + m * (np.dot(d,d) * np.eye(3) - np.outer(d,d))  # Parallel-Axis theorem
            
            # Inertia tensor about frame origin, expressed in the base frame
            Tcm = self.forward_kinematics_centre_of_mass(joint_readings, i)
            Rcm = Tcm[:3,:3]
            Jl = (Rcm @ Io) @ Rcm.T  # lecture 9 slide 12

            # Inertia matrix (lecture 9 slide 15)
            B += m * (Jp.T @ Jp) + (Jo.T @ Jl) @ Jo

        # Your code ends here ------------------------------
        
        return B

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7

        # Your code starts here ------------------------------
        
        # Initialize h & C
        h = np.zeros((7,7,7))
        C = np.zeros(7)

        q_plus = joint_readings.copy()
        eps = 1e-6

        # Lecture 9 slide 18 & lab 9 slide 13
        for i in range(7):
            for j in range(7):
                for k in range(7):

                    # Derivative
                    # db/dq = (b(q_plus)-b(q))/ eps

                    q_plus[k] += eps
                    db_ij_dq_k = (self.get_B(q_plus)[i,j] - self.get_B(joint_readings)[i,j]) / eps
                    q_plus[k] -= eps  # reset

                    q_plus[i] += eps
                    db_jk_dq_i = (self.get_B(q_plus)[j,k] - self.get_B(joint_readings)[j,k]) / eps
                    q_plus[i] -= eps  # reset

                    # h
                    h[i,j,k] = db_ij_dq_k - 0.5 * db_jk_dq_i

                    # C times qdot
                    C[i] += h[i,j,k] * joint_velocities[k]  

        # Your code ends here ------------------------------

        assert isinstance(C, np.ndarray)
        assert C.shape == (7,)
        return C

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7

        # Your code starts here ------------------------------

        g = np.zeros(7)
        eps = 1e-6
        q_plus = joint_readings.copy()

        # Lecture 9 slide 16
        for n in range(7):
            P = 0
            P_plus = 0
            
            # Potential energy P
            for i in range(7):
                Tcm = self.forward_kinematics_centre_of_mass(joint_readings, i+1)
                P -= self.mass[i] * np.dot(np.array([0,0,-self.g]), Tcm[:3,3])

            # incremented P
            q_plus[n] += eps
            for i in range(7):
                Tcm = self.forward_kinematics_centre_of_mass(q_plus, i+1)
                P_plus -= self.mass[i] * np.dot(np.array([0,0,-self.g]), Tcm[:3,3])
            
            g[n] = (P_plus - P)/eps
            q_plus[n] -= eps

        # Your code ends here ------------------------------

        assert isinstance(g, np.ndarray)
        assert g.shape == (7,)
        return g


# VERIFY 
base_robot = Iiwa14DynamicBase()
kdl_robot = Iiwa14DynamicKDL()
my_robot = Iiwa14DynamicRef()

# Joints & velocities (random values)
test_joints = [
    np.random.uniform(base_robot.joint_limit_min[i], base_robot.joint_limit_max[i]) 
    for i in range(7)
    ]
test_velocities = np.random.uniform(0.2, 1, 7).tolist()

#  Jacobian
my_jacobian = my_robot.get_jacobian_centre_of_mass(test_joints)

# B
kdl_B = kdl_robot.get_B(test_joints)
my_B = my_robot.get_B(test_joints) 
max_diff_B =  np.max(np.abs(kdl_B - my_B))
print(f"Maximum error, B: {max_diff_B}")  # < 0.1

# C_times_qdot
kdl_C = kdl_robot.get_C_times_qdot(test_joints, test_velocities)
my_C = my_robot.get_C_times_qdot(test_joints, test_velocities)
max_diff_C =  np.max(np.abs(kdl_C - my_C))
print(f"Maximum error, C times qdot: {max_diff_C}")  # < 1

# g
kdl_g = kdl_robot.get_G(test_joints)
my_g = my_robot.get_G(test_joints)
max_diff_g =  np.max(np.abs(kdl_g - my_g))
print(f"Maximum error, g: {max_diff_g}")  # < 0.05