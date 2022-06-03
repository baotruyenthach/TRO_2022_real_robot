#!/usr/bin/env python

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py
from logging import shutdown
import sys
import copy
import rospy
import moveit_commander
# import geometry_msgs.msg
# from utils import deformernet_client
import numpy as np
from std_msgs.msg import UInt16
import baxter_interface
from baxter_interface import CHECK_VERSION
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

from core import Robot
from copy import deepcopy
import transformations
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from trac_ik_python.trac_ik import IK

from deformernet_utils import deformernet_client


"""To modify baxter.urdf
1) Measure length L of the baxter from the end of the end-effector (left_gripper_base link) (the part right before the left/right parralel grippers
2) Modify "laparoscopic_tool" (both visual and collision) <box size="L 0.005 0.005" />
3) Modify "laparoscopic_tool" origins -> L/2 + 0.018; 0.018: some offset from the left_gripper_base to the actual end of ee; e.g. if L = 0.3: <origin xyz="0 0 0.168" rpy="0 1.570796327 0 " />
4) Mdoify "laparoscopic_tool_tip" <joint name="dummy_joint" type="fixed">: <origin rpy="0 0 0" xyz="0.0 0.0 (L + 0.018)"/>
"""




def obtain_waypoints(current_pose, delta_pos, delta_euler, num_wp=10):
    """ Return list of waypoints interpolated between init and goal pose
    Interpolation in angles using SLERP """
    # euler order: standing opposite - roll, pitch, yaw
    
    current_euler = transformations.euler_from_quaternion([current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w])
    goal_euler = np.array(delta_euler) + np.array(current_euler)
    
    euler = np.concatenate((np.array([current_euler]), [goal_euler]), axis=0)
    key_rots = R.from_euler('xyz', euler)
    key_times = [0,1]
    slerp = Slerp(key_times, key_rots)
    times = np.linspace(start=0,stop=1,num=num_wp) 
    interp_rots = slerp(times)
    # print(key_rots.as_euler('xyz'), interp_rots.as_euler('xyz'))
    interp_eulers = interp_rots.as_euler('xyz')
    # print("interp_eulers:", interp_eulers)

    interp_pos = np.linspace(start=[0,0,0],stop=delta_pos,num=num_wp)
    # print("interp_pos:", interp_pos)

    target_pose = deepcopy(current_pose)
    waypoints = []

    for i in range(1, num_wp):
        
        quat = transformations.quaternion_from_euler(*interp_eulers[i].squeeze())
        
        target_pose.position.x = current_pose.position.x + interp_pos[i][0]
        target_pose.position.y = current_pose.position.y + interp_pos[i][1]
        target_pose.position.z = current_pose.position.z + interp_pos[i][2]   


        
        target_pose.orientation.x = quat[0]     
        target_pose.orientation.y = quat[1]    
        target_pose.orientation.z = quat[2]    
        target_pose.orientation.w = quat[3]    
        
        waypoints.append(copy.deepcopy(target_pose))

    # print("----------------------------")
    # print("init:", current_pose)
    # print("----------------------------")
    # print("final:", waypoints[-1])
    return waypoints

def moveit_baxter_example(delta_pos, delta_euler, num_wp=10):
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    # rospy.init_node('moveit_baxter_example', anonymous=True)


    """ First. Obtain the new pose of "laparoscopic_tool_tip" (after adding the desired displacements) using group.get_current_pose()
    Then, get desired joint angles with track_ik"""
    
    ### Set up
    group = moveit_commander.MoveGroupCommander("left_arm")  
    ik_solver = IK("world", "laparoscopic_tool_tip")
    # print(ik_solver.joint_names)
    # seed_state = [0.0] * ik_solver.number_of_joints
    arm = baxter_interface.Limb('left')
    robot_description = URDF.from_parameter_server()
    base_link = 'world'
    end_link = 'laparoscopic_tool_tip'
    kdl_kin = KDLKinematics(robot_description, base_link, end_link)
    robot = Robot(arm, kdl_kin)

    ### Add displacements to current pose
    left_current_pose = deepcopy(group.get_current_pose(end_effector_link='laparoscopic_tool_tip').pose) # current pose of "laparoscopic_tool_tip"    
    eulers = transformations.euler_from_quaternion([left_current_pose.orientation.x, left_current_pose.orientation.y, left_current_pose.orientation.z, 
                                                    left_current_pose.orientation.w])
    eulers_new = np.array(eulers) + np.array(delta_euler)
    quat = transformations.quaternion_from_euler(*eulers_new)

    ### Solve IK to obtain desired joint positions. 
    seed_state = group.get_current_joint_values()
    ik_js = ik_solver.get_ik(seed_state, left_current_pose.position.x + delta_pos[0], left_current_pose.position.y + delta_pos[1], \
                            left_current_pose.position.z + delta_pos[2], \
                            *quat) 

    ### Optional. Use KDLKinematics to find the goal pose to confirm the solution is correct.
    pos, temp = robot.get_left_gripper_from_joint_angles(np.array(ik_js))
    euler = np.array([temp[2], -temp[1], -np.pi-temp[0]])   # Modify the eulers a bit to match the results between KDLKinematics and group.get_current_pose()   
    print("===================================")
    print([left_current_pose.position.x,left_current_pose.position.y,left_current_pose.position.z], np.array(eulers)*180/np.pi)
    print(pos, euler*180/np.pi)
    print("===================================")    


    """ Second. Compute the desired pose of the "left_gripper" link using FK. Then execute the goal pose with MoveIt"""

    end_link = 'left_gripper'
    kdl_kin = KDLKinematics(robot_description, base_link, end_link)
    robot = Robot(arm, kdl_kin)
    
    ### Compute the desired pose of the "left_gripper" link using FK
    pos, temp = robot.get_left_gripper_from_joint_angles(np.array(ik_js))
    euler = np.array([temp[2], -temp[1], -np.pi-temp[0]])


    ### Compute deltas pos and eulers from the current pose to the desired pose.
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    
    delta_position = pos - np.array([left_current_pose.position.x,left_current_pose.position.y,left_current_pose.position.z])
    current_euler = transformations.euler_from_quaternion([left_current_pose.orientation.x, left_current_pose.orientation.y, left_current_pose.orientation.z, 
                                                            left_current_pose.orientation.w])
    delta_eulers = euler - current_euler  
    # print([left_current_pose.position.x,left_current_pose.position.y,left_current_pose.position.z], np.array(current_euler)*180/np.pi)                                                          
    # print("xxxxxxxxxxxdelta_pos, delta_euler:", delta_position, delta_eulers*180/np.pi)


    ### Get waypoints to execute MoveIt Catersian Path. Reference: http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
    waypoints = obtain_waypoints(left_current_pose, delta_position, delta_eulers, num_wp)


    ### Execute MoveIt
    (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    

    if not plan.joint_trajectory.points:
        print "[ERROR] No trajectory found"
    else:
        group.execute(plan, wait=True)

    return None
    # return np.array([left_current_pose.position.x, left_current_pose.position.y, left_current_pose.position.z, 
    #                 left_current_pose.orientation.x, left_current_pose.orientation.y,
    #                 left_current_pose.orientation.z, left_current_pose.orientation.w])



if __name__ == '__main__':

    rospy.init_node('moveit_baxter_example', anonymous=True)


    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()


    ### Set up 
    arm = baxter_interface.Limb('left')
    robot_description = URDF.from_parameter_server()
    base_link = 'world'
    # end_link = 'left_gripper'
    end_link = 'laparoscopic_tool_tip'
    kdl_kin = KDLKinematics(robot_description, base_link, end_link)
    robot = Robot(arm, kdl_kin)

    ### Desired displacements
    delta_pos = np.array([-0.0, -0.0, -0.1])
    delta_euler = np.array([0, 0, -np.pi/4]) # euler order: standing opposite - roll, pitch, yaw

    rospy.sleep(1)
    init_ee = robot.get_ee_cartesian_position()   # Cache initial end-effector pose

    try:
        final_ee = moveit_baxter_example(delta_pos, delta_euler, num_wp=2)
    except rospy.ROSInterruptException:
        pass

    rospy.sleep(1)
    final_ee = robot.get_ee_cartesian_position()   # Cache final end-effector pose
    
    ### Compute actual dispalcements to confirm that the robot is operating properly.
    print("diff_ee pos:", final_ee[:3] - init_ee[:3])
    print("diff_ee euler:", 180/np.pi*np.array(transformations.euler_from_quaternion(final_ee[3:])) - 180/np.pi*np.array(transformations.euler_from_quaternion(init_ee[3:])))
    print("----------------------------")
    print("final_ee pos:", final_ee[:3])
    print("final_ee quat:", final_ee[3:])



