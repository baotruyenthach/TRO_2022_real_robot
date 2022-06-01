import numpy as np
import torch
import os
import ctypes
import struct
from sensor_msgs import point_cloud2
from point_cloud_segmentation.srv import *

import rospy
# import open3d
import moveit_commander
import copy

import sys
# sys.path.append('/home/briancho/shape_servo_DNN')
# from farthest_point_sampling import *

# sys.path.append('/home/briancho/shape_servo_DNN/generalization_tasks')
# from architecture import DeformerNet2, DeformerNet


def deformernet_client():
    rospy.loginfo('Waiting for service get_dn_baxter.')
    rospy.wait_for_service('get_dn_baxter')
    rospy.loginfo('Calling service get_dn_baxter.')
    try:
        dn_baxter_proxy = rospy.ServiceProxy('object_segmenter', SegmentGraspObject)
        dn_baxter_request = SegmentGraspObjectRequest()
        dn_baxter_response = dn_baxter_proxy(object_segment_request) 

    except (rospy.ServiceException, e):
        rospy.loginfo('Service object_segmenter call failed: %s'%e)
    rospy.loginfo('Service object_segmenter is executed.')
    if not object_segment_response.object_found:
        rospy.logerr('No object found from segmentation!')
        return False
    return object_segment_response.obj

def moveit_baxter(pos):
    print("Before clamping: ", pos)
    pos[0] = np.clip(pos[0], -0.1, 0.1)
    pos[1] = np.clip(pos[1], 0, 0.1)
    pos[2] = np.clip(pos[2], 0, 0.1)
    print("After clamping: ", pos)
    try:
        joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        #rospy.init_node('moveit_baxter_example', anonymous=True)    
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander("right_arm")

        right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose
        waypoints = []
        right_target_pose = right_current_pose

        right_target_pose.position.x = right_current_pose.position.x + pos[0]
        right_target_pose.position.y = right_current_pose.position.y + pos[1]
        right_target_pose.position.z = right_current_pose.position.z + pos[2]
        waypoints.append(copy.deepcopy(right_target_pose))

        (plan, fraction) = group.compute_cartesian_path(
                                                        waypoints,   # waypoints to follow
                                                        0.01,        # eef_step
                                                        0.0)         # jump_threshold
        if not plan.joint_trajectory.points:
            print("[ERROR] No trajectory found")
        else:
            #group.go(wait=True)
            group.execute(plan, wait=True)
    except (rospy.ServiceException, e):
        rospy.loginfo('Moveit Baxter call failed: %s'%e)

def segment_object_client():  
    '''
    segment object out from the image and assign to self.object_segment_response
    '''
    rospy.loginfo('Waiting for service object_segmenter.')
    rospy.wait_for_service('object_segmenter')
    rospy.loginfo('Calling service object_segmenter.')
    try:
        object_segment_proxy = rospy.ServiceProxy('object_segmenter', SegmentGraspObject)
        object_segment_request = SegmentGraspObjectRequest()
        object_segment_response = object_segment_proxy(object_segment_request) 

    except (rospy.ServiceException, e):
        rospy.loginfo('Service object_segmenter call failed: %s'%e)
    rospy.loginfo('Service object_segmenter is executed.')
    if not object_segment_response.object_found:
        rospy.logerr('No object found from segmentation!')
        return False
    return object_segment_response.obj

def store_rgb_cloud(pc2_msg):

    color = []
    rgb_points = point_cloud2.read_points(pc2_msg, skip_nans=True, field_names=("rgb"))
    for data in rgb_points:            

        s = struct.pack('>f' ,data)
        i = struct.unpack('>l',s)[0]
        pack = ctypes.c_uint32(i).value
        r = int((pack & 0x00FF0000)>> 16)
        g = int((pack & 0x0000FF00)>> 8)
        b = int((pack & 0x000000FF))

        color.append([r, g, b])
    return np.array(color)


class DeformerNetBaxter():
    def __init__(self, goal_pc=None, model_type=1):
        self.model_type = model_type
        self.goal_pc = goal_pc      # array shape [N,3]
        # self.setup_deformernet()       
    
    def setup_deformernet(self):
        if self.model_type == 1: 
            weight_path = "/home/briancho/shape_servo_data/real_robot/model/single box"
            self.model = DeformerNet2(normal_channel=False)
        elif self.model_type == 2: 
            weight_path = "/home/briancho/shape_servo_data/real_robot/model/multi boxes"
            self.model = DeformerNet2(normal_channel=False)
        self.model.load_state_dict(weight_path)  
        self.model.eval()

    def get_current_pc(self, filter_by_color=True, intensity_thres=0.4, visualization=True):
        segmented_obj = segment_object_client()

        points_xyz = list(point_cloud2.read_points(segmented_obj.cloud,field_names=['x','y','z']))
        

        if filter_by_color:
            points_color = store_rgb_cloud(segmented_obj.cloud)
            points_color = points_color/255.    # convert to range [0,1]
            xyz = []
            # color = []
            for i, point in enumerate(points_color):
                if all(np.array(point) > intensity_thres):
                    xyz.append(points_xyz[i])
                    # color.append(point)
            points_xyz = xyz 
            # points_color = color  

        # pcd = open3d.geometry.PointCloud()
        # pcd.points = open3d.utility.Vector3dVector(np.array(points_xyz)) 
        # pcd_goal = open3d.geometry.PointCloud()
        # pcd_goal.points = open3d.utility.Vector3dVector(self.goal_pc)  
        # self.chamfer_dist = np.linalg.norm(np.asarray(pcd_goal.compute_point_cloud_distance(pcd)))
        # rospy.logwarn("Current chamfer:", self.chamfer_dist)

        # if visualization:
        #     open3d.visualization.draw_geometries([pcd, pcd_goal])   

        self.current_pc = np.array(points_xyz)
        # return np.array(points_xyz)

    def run_deformernet(self):
        # current_pc = self.format_pc(self.current_pc)
        # goal_pc = self.format_pc(self.goal_pc)
        # with torch.no_grad():
        #     desired_position = self.model(current_pc.unsqueeze(0), goal_pc.unsqueeze(0))[0].detach().numpy()*(0.001)  
        desired_position = [0.02, 0.02, 0.02]
        print(" === desired_position: ", desired_position)

    def format_pc(self, pc):
        farthest_indices,_ = farthest_point_sampling(pc, 1024)
        pc = pc[farthest_indices.squeeze()]    
        pc = torch.from_numpy(pc).permute(1,0).float()     

         



