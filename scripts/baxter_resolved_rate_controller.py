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


def set_neutral():
    """
    Sets both arms back into a neutral pose.
    """
    print("Moving to neutral pose...")
    # zeros_pos = dict([(joint, 0) for i, joint in enumerate(baxter_interface.Limb('left').joint_names())])
    angles_left = left_arm.joint_angles()
    angles_right = right_arm.joint_angles()
    left_arm.move_to_joint_positions(positions=angles_left)
    right_arm.move_to_joint_positions(positions=angles_right)
    # left_arm.move_to_neutral(timeout=2.0)
    # right_arm.move_to_neutral(timeout=2.0)
    # vel_cmd = dict([(joint, 0) for i, joint in enumerate(left_arm.joint_names())])
    # left_arm.set_joint_velocities(vel_cmd)

def clean_shutdown():
    print("\nExiting example...")
    #return to normal

    set_neutral()



rospy.init_node('baxter_resolved_rate_controller')
left_arm = baxter_interface.limb.Limb("left")
right_arm = baxter_interface.limb.Limb("right")

# rospy.sleep(1)
# angles_left = left_arm.joint_angles()
# angles_right = right_arm.joint_angles()

arm = baxter_interface.Limb('left')
robot_description = URDF.from_parameter_server()
base_link = 'world'
end_link = 'left_gripper'
kdl_kin = KDLKinematics(robot_description, base_link, end_link)
robot = Robot(arm, kdl_kin)

desired_position = np.array([-0.0, -0.3, 0.2])
# desired_position = np.array([0.0, -0.4, -0.0])


rs = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = rs.state().enabled
print("Enabling robot... ")
rs.enable()

rospy.on_shutdown(clean_shutdown)

pub_rate = rospy.Publisher('robot/joint_state_publish_rate', UInt16, queue_size=10)
pub_rate.publish(100)    # the joint state publishing rate is set to be at 100 Hz.
action = 'init'
rate = rospy.Rate(60)   # 60 Hz loops
while not rospy.is_shutdown():    
# for i in range(100):
    if action is 'init':
        tvc_behavior = TaskVelocityControl(desired_position, robot, vel_limits=robot.vel_limits)
    
    elif tvc_behavior.is_complete_success():
        rospy.logwarn("DONE!!!")
        break
    else: 
         
        
        robot.set_dof_velocity_targets(action)
        rate.sleep()

    action = tvc_behavior.get_action()



    # print("desired_vel:", action)

