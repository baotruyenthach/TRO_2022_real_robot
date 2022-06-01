#!/usr/bin/env python

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
#import utils.deformernet_client
#import numpy as np


def moveit_baxter_example():
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    rospy.loginfo('Waiting for service object_segmenter.')
    rospy.wait_for_service('object_segmenter')
    rospy.loginfo('Calling service object_segmenter.')
    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("right_arm")

    # Planning to a Pose goal
    #left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose
    waypoints = []
    #left_target_pose = left_current_pose
    #left_target_pose.position.x = left_current_pose.position.x - 0.1 # 0.1m = 10 cm
    #left_target_pose.position.z = left_current_pose.position.z + 0.2
    #waypoints.append(copy.deepcopy(left_target_pose))

    #left_target_pose.position.z = left_current_pose.position.z + 0.1
    #waypoints.append(copy.deepcopy(left_target_pose))

    #left_target_pose.position.z = left_current_pose.position.z - 0.15
    #waypoints.append(copy.deepcopy(left_target_pose))
    right_target_pose = right_current_pose
    #right_target_pose.position.x = right_current_pose.position.x - 0.15
    right_target_pose.position.y = right_current_pose.position.y - 0.05
    right_target_pose.position.z = right_current_pose.position.z + 0.15
    waypoints.append(copy.deepcopy(right_target_pose))

    print "On the waypoint 1: \n", right_target_pose
    #group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    #group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

    #right_target_pose.position.x = right_current_pose.position.x + 0.05
    #right_target_pose.position.z = right_current_pose.position.z + 0.1
    #waypoints.append(copy.deepcopy(right_target_pose))

    #right_target_pose.position.x = right_current_pose.position.x - 0.05
    #right_target_pose.position.z = right_current_pose.position.z - 0.2
    #waypoints.append(copy.deepcopy(right_target_pose))

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
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        moveit_baxter_example()
    except rospy.ROSInterruptException:
        pass
