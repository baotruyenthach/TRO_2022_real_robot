import rospy
import numpy as np

import sys
import transformations

from copy import deepcopy

class Robot:
    """Baxter robot right arm"""

    def __init__(self, arm, kdl_kin):
        self.arm = arm  # baxter arm obtained from baxter_interface.limb.Limb("right")
        self.kdl_kin = kdl_kin
        self.joint_names = self.arm.joint_names()
        self.vel_limits = np.array([1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0])/6   # right or left arm Baxter; 4.0s are w0, w1, w2
        # print("joint names:", self.joint_names)
        self.n_arm_dof = 7

    def get_arm_joint_positions(self):
        joint_angles = deepcopy(self.arm.joint_angles())
        return [joint_angles[joint] for joint in self.joint_names]
        # return [joint_angles['right_e0'], joint_angles['right_e1'], joint_angles['right_s0'], \
        #         joint_angles['right_s1'], joint_angles['right_w0'], joint_angles['right_w1'], joint_angles['right_w2']]



    def get_ee_cartesian_position(self):
        """
        7-dimension pos + rot
        """
        pose = np.zeros(7)
        q_cur = self.get_arm_joint_positions()
        # print(q_cur)
        pose_matrix = np.asarray(self.kdl_kin.forward(q_cur))   # np matrix -> np array
        # quat = transformations.quaternion_from_matrix(pose_matrix[:3,:3])

        pose[:3] = deepcopy(pose_matrix[:3,3])
        pose[3:] = transformations.quaternion_from_matrix(pose_matrix)
        # quaternion rotation just set to zeros as it doesn't matter
        # tf_matrix[3:] = quat   
        return pose


    def set_dof_velocity_targets(self, vels):
        """       

        Args:
            vel ([list or np array]):   desired joint velocities
        """
        # print("upper lims:", self.kdl_kin.joint_limits_upper) #1.70167994, 1.047, 3.05417994, 2.618, 3.059, 2.094, 3.059
        # print("lower lims:", self.kdl_kin.joint_limits_lower) #-1.70167994, -2.147, -3.05417994, -0.05, -3.059, -1.57079633,-3.059

        
        vel_cmd = dict([(joint, vels[i]) for i, joint in enumerate(self.joint_names)])
        self.arm.set_joint_velocities(vel_cmd)


    def get_left_gripper_from_joint_angles(self, joint_angles):
        """ Compute pos and orientation (euler angles) of left gripper using the joint angles """
        pose_matrix = np.asarray(self.kdl_kin.forward(joint_angles))
        pos = deepcopy(pose_matrix[:3,3])
        euler = np.array(transformations.euler_from_matrix(pose_matrix))
        return pos, euler


        



