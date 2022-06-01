#!/usr/bin/env python


import rospy

from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs.msg import JointState
#from std_msgs.msg import Char
from sensor_msgs.msg import JointState
import moveit_msgs.msg
import moveit_commander
#import geometry_msgs.msg
#from geometry_msgs.msg import PoseStamped,Pose
import numpy as np
from trac_ik_python.trac_ik import IK

import baxter_interface
from baxter_interface import CHECK_VERSION
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from core import Robot
from copy import deepcopy
import transformations
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


rospy.init_node('trackik', anonymous=False)


rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
print("Enabling robot... ")
rs.enable()

# robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()


joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)

group = moveit_commander.MoveGroupCommander('left_arm')
# group.set_planner_id('RRTConnectkConfigDefault')
group.set_planner_id('RRTstarkConfigDefault')
group.set_planning_time(10)
group.set_num_planning_attempts(3)

ik_solver = IK("world", "laparoscopic_tool_tip")
print(ik_solver.joint_names)
seed_state = [0.0] * ik_solver.number_of_joints

arm = baxter_interface.Limb('left')
robot_description = URDF.from_parameter_server()
base_link = 'world'
end_link = 'laparoscopic_tool_tip'
# end_link = 'laparoscopic_tool_tip'
kdl_kin = KDLKinematics(robot_description, base_link, end_link)
robot = Robot(arm, kdl_kin)

# rospy.sleep(1)
# init_ee = robot.get_ee_cartesian_position()
# print(init_ee)
group.clear_pose_targets()

print("===================================")
left_current_pose = group.get_current_pose(end_effector_link='laparoscopic_tool_tip').pose
# print(left_current_pose)
# left_current_pose = deepcopy(group.get_current_pose(end_effector_link='left_gripper').pose)
# print(left_current_pose)
init_ee = robot.get_ee_cartesian_position()
print(init_ee)
print("===================================")
# ik_js = ik_solver.get_ik(seed_state, *init_ee)
# ik_js = ik_solver.get_ik(seed_state, init_ee[0], init_ee[1], init_ee[2], 0.0, 0.0, 0.0, 1.0)
# ik_js = ik_solver.get_ik(seed_state, 0.45, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0)
# ik_js = ik_solver.get_ik(seed_state, left_current_pose.position.x, left_current_pose.position.y, left_current_pose.position.z,
#                                 left_current_pose.orientation.x, left_current_pose.orientation.y, left_current_pose.orientation.z, 
#                                 left_current_pose.orientation.w)


seed_state = group.get_current_joint_values()
eulers = transformations.euler_from_quaternion([left_current_pose.orientation.x, left_current_pose.orientation.y, left_current_pose.orientation.z, 
                                                left_current_pose.orientation.w])
eulers_new = np.array(eulers) + np.array([np.pi/100,0, 0])
quat = transformations.quaternion_from_euler(*eulers_new)

ik_js = ik_solver.get_ik(seed_state, left_current_pose.position.x, left_current_pose.position.y, left_current_pose.position.z+0.1,
                         *quat)                                
print(ik_js)


group.set_joint_value_target(np.array(ik_js))
plan_goal = group.plan()
# print(plan_goal)
group.execute(plan_goal, wait=True)


print("===================================")
# left_current_pose = group.get_current_pose(end_effector_link='laparoscopic_tool_tip').pose
# print(left_current_pose)
# print(robot.get_ee_cartesian_position())
# left_current_pose = deepcopy(group.get_current_pose(end_effector_link='left_gripper').pose)
# print(left_current_pose)
init_ee = robot.get_ee_cartesian_position()
print(init_ee)
print("===================================")


