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


home_left = {'left_w0': -0.44787147919529247, 'left_w1': -0.030359275227683113, 'left_w2': -0.050288503724890354, 'left_e0': 0.007414220958978035, 'left_e1': 0.5036717340443388, 'left_s0': 0.19083392898149576, 'left_s1': 1.0470047162936567}
home_right = {'right_s0': -0.35752707674516415, 'right_s1': 1.047003659427773, 'right_w0': -0.27829191032360523, 'right_w1': 0.0357530468626619, 'right_w2': 0.11129875866436478, 'right_e0': 0.006869317940742192, 'right_e1': 0.5046978395373198}


def obtain_waypoints(current_pose, delta_pos, delta_euler, num_wp=10):
    """ Return list of waypoints interpolated between init and goal pose
    Interpolation in angles using SLERP """
    # euler order: standing opposite - roll, pitch, yaw
    
    current_euler = transformations.euler_from_quaternion([current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w])
    goal_euler = np.array(delta_euler) + np.array(current_euler)
    

    key_rots = R.from_euler(np.concatenate((current_euler, goal_euler)))
    key_times = [0,1]
    slerp = Slerp(key_times, key_rots)
    times = np.linspace(start=0,stop=1,num=num_wp) #range(0,10)
    interp_rots = slerp(times)
    # print(key_rots.as_euler('xyz'), interp_rots.as_euler('xyz'))
    interp_eulers = interp_rots.as_euler('xyz')

    interp_pos = np.linspace(start=[0,0,0],stop=delta_pos,num=num_wp)

    target_pose = current_pose  
    waypoints = []

    for i in range(1, num_wp):
        
        quat = transformations.quaternion_from_euler(*interp_eulers[i])
        
        target_pose.position.x = target_pose.position.x + interp_pos[i][0]
        target_pose.position.y = target_pose.position.y + interp_pos[i][1]
        target_pose.position.z = target_pose.position.z + interp_pos[i][2]        

        
        target_pose.orientation.x = quat[0]    #current_pose.orientation.x +  
        target_pose.orientation.y = quat[1]    #current_pose.orientation.y + 
        target_pose.orientation.z = quat[2]    #current_pose.orientation.z + 
        target_pose.orientation.w = quat[3]    #current_pose.orientation.w + 
        
        waypoints.append(copy.deepcopy(target_pose))

    return waypoints

def moveit_baxter_example():
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    # rospy.init_node('moveit_baxter_example', anonymous=True)


    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    # pos = deformernet_client()
    

    # # Brian stuff
    # pos[0] = np.clip(pos[0], -0.1, 0.1)
    # pos[1] = np.clip(pos[1], 0, 0.1)
    # pos[2] = np.clip(pos[2], 0, 0.1)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("left_arm")    


    # Planning to a Pose goal
    #left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    waypoints = []



    #left_target_pose = left_current_pose
    #left_target_pose.position.x = left_current_pose.position.x - 0.1 # 0.1m = 10 cm
    #left_target_pose.position.z = left_current_pose.position.z + 0.2
    #waypoints.append(copy.deepcopy(left_target_pose))

    #left_target_pose.position.z = left_current_pose.position.z + 0.1
    #waypoints.append(copy.deepcopy(left_target_pose))

    #left_target_pose.position.z = left_current_pose.position.z - 0.15
    #waypoints.append(copy.deepcopy(left_target_pose))
    left_target_pose = left_current_pose
    left_target_pose.position.x = left_current_pose.position.x + pos[0]
    left_target_pose.position.y = left_current_pose.position.y + pos[1]
    left_target_pose.position.z = left_current_pose.position.z + pos[2]
    
    
    
    euler_current = transformations.euler_from_quaternion([left_current_pose.orientation.x,left_current_pose.orientation.y,left_current_pose.orientation.z,left_current_pose.orientation.w])
    new_euler = np.array(euler) + np.array(euler_current)
    quat = transformations.quaternion_from_euler(*new_euler)
    left_target_pose.orientation.x = quat[0]    #left_current_pose.orientation.x +  
    left_target_pose.orientation.y = quat[1]    #left_current_pose.orientation.y + 
    left_target_pose.orientation.z = quat[2]    #left_current_pose.orientation.z + 
    left_target_pose.orientation.w = quat[3]    #left_current_pose.orientation.w + 
    waypoints.append(copy.deepcopy(left_target_pose))

    # print(waypoints)
    print("----------------------------")
    print("init:", left_current_pose)
    print("----------------------------")
    print("final:", waypoints[-1])

    # print "On the waypoint 1: \n", left_target_pose
    #group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    #group.set_pose_target(left_target_pose, end_effector_link='left_gripper')

    #left_target_pose.position.x = left_current_pose.position.x + 0.05
    #left_target_pose.position.z = left_current_pose.position.z + 0.1
    #waypoints.append(copy.deepcopy(left_target_pose))

    #left_target_pose.position.x = left_current_pose.position.x - 0.05
    #left_target_pose.position.z = left_current_pose.position.z - 0.2
    #waypoints.append(copy.deepcopy(left_target_pose))

    (plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
    
    #plan = group.plan()

    if not plan.joint_trajectory.points:
        print "[ERROR] No trajectory found"
    else:
        #group.go(wait=True)
        group.execute(plan, wait=True)
    # When finished shut down moveit_commander.
    # moveit_commander.roscpp_shutdown()
    # moveit_commander.os._exit(0)
    return np.array([left_current_pose.position.x, left_current_pose.position.y, left_current_pose.position.z])



if __name__ == '__main__':

    rospy.init_node('moveit_baxter_example', anonymous=True)


    # robot = moveit_commander.RobotCommander()
    # group = moveit_commander.MoveGroupCommander("left_arm")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    left_arm = baxter_interface.limb.Limb("left")
    right_arm = baxter_interface.limb.Limb("right")


    arm = baxter_interface.Limb('left')
    robot_description = URDF.from_parameter_server()
    base_link = 'world'
    end_link = 'left_gripper'
    kdl_kin = KDLKinematics(robot_description, base_link, end_link)
    robot = Robot(arm, kdl_kin)
    print()

    pos = np.array([-0.1, -0.1, -0.1])
    # pos = np.array([-0.1, -0.4, -0.1]) #np.array([0.00, -0.0, 0.0])
    euler = np.array([0, 0, -np.pi/4]) #[0,0,np.pi/6]   # standing opposite: roll, pitch, yaw

    rospy.sleep(1)
    init_ee = robot.get_ee_cartesian_position()
    # pose = deepcopy(group.get_current_pose(end_effector_link='left_gripper').pose.position)
    # init_ee = np.array([pose.x, pose.y, pose.z])
    





    # angles_left = left_arm.joint_angles()
    # angles_right = right_arm.joint_angles()    


    # print(angles_left)
    # print("-----------------")
    # print(angles_right)

    try:
        moveit_baxter_example()
    except rospy.ROSInterruptException:
        pass


    # robot = moveit_commander.RobotCommander()
    # group = moveit_commander.MoveGroupCommander("left_arm")

    rospy.sleep(1)
    final_ee = robot.get_ee_cartesian_position()
    # pose = deepcopy(group.get_current_pose(end_effector_link='left_gripper').pose.position)
    # final_ee = np.array([pose.x, pose.y, pose.z])
    print("diff_ee pos:", final_ee[:3] - init_ee[:3])
    print("diff_ee euler:", np.array(transformations.euler_from_quaternion(final_ee[3:])) - np.array(transformations.euler_from_quaternion(init_ee[3:])))

    # print("final_ee:", final_ee)
    # print("init_ee:", init_ee)



    # rospy.sleep(5)
    # rs.disable()
    # rospy.sleep(1)
    # left_arm.move_to_joint_positions(positions=angles_left)
    # right_arm.move_to_joint_positions(positions=angles_right)

    # shutdown()
