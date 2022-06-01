#!/usr/bin/env python
import sys
from turtle import position
import roslib.packages as rp
import rospy
import numpy as np

pkg_path = rp.get_pkg_dir('baxter_moveit_tutorial')
sys.path.append(pkg_path + '/src')

from core import Robot
from behaviors import TaskVelocityControl
import baxter_interface
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from std_msgs.msg import UInt16

from baxter_interface import CHECK_VERSION


# def clean_shutdown():
#     left_arm.move_to_joint_positions(positions=home_left)
#     right_arm.move_to_joint_positions(positions=home_right)


rospy.init_node('home_position')
rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
print("Enabling robot... ")
rs.disable()
rospy.sleep(2)
rs.enable()

left_arm = baxter_interface.limb.Limb("left")
right_arm = baxter_interface.limb.Limb("right")


# home_left = dict([(joint, 0) for i, joint in enumerate(left_arm.joint_names())])
# # home_left = {'left_w0': -0.6577515300810433, 'left_w1': 1.6484066320675579, 'left_w2': -1.0516304629456297, 'left_e0': 0.7610965505349965, 'left_e1': 0.39114585960614434, 'left_s0': -0.29500178423823353, 'left_s1': 0.29697050105877043}

# home_right = {'right_s0': -0.3561298583298287, 'right_s1': 1.0470026800072434, 'right_w0': -0.28678799613079153, 'right_w1': 0.03446691916440958, 'right_w2': 0.10859384029469066, 'right_e0': 0.007019199325839054, 'right_e1': 0.5045651916408769}
home_pos = np.array([-0.6, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0])
home_left = dict([(joint, home_pos[i]) for i, joint in enumerate(left_arm.joint_names())])
# home_right = dict([(joint, home_pos[i]) for i, joint in enumerate(right_arm.joint_names())])


# rospy.on_shutdown(clean_shutdown)



left_arm.move_to_joint_positions(positions=home_left, timeout=2.0)
# right_arm.move_to_joint_positions(positions=home_right, timeout=2)


# left_arm.move_to_neutral(timeout=2.0)
right_arm.move_to_neutral(timeout=2.0)
# vel_cmd = dict([(joint, 0) for i, joint in enumerate(left_arm.joint_names())])
# left_arm.set_joint_velocities(vel_cmd)

# rospy.spin()