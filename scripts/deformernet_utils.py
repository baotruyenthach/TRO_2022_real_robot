import numpy as np
import torch
import os
import ctypes
import struct
from sensor_msgs import point_cloud2
from point_cloud_segmentation.srv import *
import rospy
import open3d
# import moveit_commander
import copy
from goal_plane import *
from copy import deepcopy
import timeit
import sys
sys.path.append('/home/baothach/shape_servo_DNN')
from farthest_point_sampling import *
from deformernet_plus_real_robot.srv import *



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

        s = struct.pack('>f' ,data[0])
        i = struct.unpack('>l',s)[0]
        pack = ctypes.c_uint32(i).value
        r = int((pack & 0x00FF0000)>> 16)
        g = int((pack & 0x0000FF00)>> 8)
        b = int((pack & 0x000000FF))

        color.append([r, g, b])
    return np.array(color)


class DeformerNetBaxter():
    def __init__(self, goal_pc=None, model_type=1, prob_type="goal_oriented", constrain_plane=None):
        self.model_type = model_type
        self.prob_type = prob_type
        self.goal_pc = goal_pc      # array shape [N,3]
        self.constrain_plane = constrain_plane
        if constrain_plane is not None:
            self.plan_vis = vis_constrain_plane(constrain_plane)
        self.setup_deformernet()       
        self.count = 0
    
    def setup_deformernet(self):
        
        if self.prob_type == "goal_oriented":
            sys.path.append('/home/baothach/shape_servo_DNN/generalization_tasks')
            from architecture import DeformerNet2, DeformerNet
            if self.model_type == 1: 
                weight_path = "/home/baothach/shape_servo_data/RL_shapeservo/box/weights/run1epoch 300"
                self.model = DeformerNet2(normal_channel=False)
        elif self.prob_type == "goal_oriented_orientation":
            sys.path.append('/home/baothach/shape_servo_DNN/rotation')
            from architecture import DeformerNetMP
            weight_path = "/home/baothach/shape_servo_data/RL_shapeservo/cylinder_orientation/weights/epoch 300"
            self.model = DeformerNetMP(normal_channel=False)            

            # elif self.model_type == 2: 
            #     weight_path = "/home/baothach/shape_servo_data/real_robot/weights/epoch 240"
            #     self.model = DeformerNet(normal_channel=False)
        elif self.prob_type == "plane":
            weight_path = "/home/baothach/shape_servo_data/RL_shapeservo/box/weights/run1epoch 300"
            self.model = DeformerNet2(normal_channel=False)            
            self.get_current_pc(filter_by_color=True, intensity_thres=0.6, visualization=False, get_init_plane_cloud=True)
            self.goal_pc = get_goal_plane(constrain_plane=self.constrain_plane, initial_pc=self.current_pc)  
        self.model.load_state_dict(torch.load(weight_path))  
        self.model.eval()

        self.goal_pc = self.down_sampling(self.goal_pc)

    def get_current_pc(self, filter_by_color=True, intensity_thres=0.4, visualization=True, min_num_points=700, get_init_plane_cloud=False):
        start_time = timeit.default_timer()
        segmented_obj = segment_object_client()
        print("time:", timeit.default_timer()-start_time)

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
        # print("time:", timeit.default_timer()-start_time)
        # if len(points_xyz) <= min_num_points:
        #     rospy.logerr("Invalid point cloud")
        #     return False 

        points_xyz = self.down_sampling(np.array(points_xyz))
        if get_init_plane_cloud == False:
            pcd = open3d.geometry.PointCloud()
            pcd.points = open3d.utility.Vector3dVector(np.array(points_xyz))         
            pcd_goal = open3d.geometry.PointCloud()
            pcd_goal.points = open3d.utility.Vector3dVector(self.goal_pc)  

            if self.prob_type == "goal_oriented":            
                self.chamfer_dist = np.linalg.norm(np.asarray(pcd_goal.compute_point_cloud_distance(pcd)))
                rospy.logwarn(f"Current chamfer: {self.chamfer_dist}")

            elif self.prob_type == "plane":
                plane_success(self.constrain_plane, np.array(points_xyz))

            if visualization:
                pcd.paint_uniform_color([0,0,1])
                pcd_goal.paint_uniform_color([1,0,0])
                if self.prob_type == "goal_oriented": 
                    open3d.visualization.draw_geometries([pcd, pcd_goal])   
                elif self.prob_type == "plane":
                    open3d.visualization.draw_geometries([pcd, pcd_goal, self.plan_vis]) 

        self.current_pc = points_xyz #np.array(points_xyz)
        print("time:", timeit.default_timer()-start_time)
        return True
        # return np.array(points_xyz)

    def run_deformernet(self):
        current_pc = self.format_pc(self.current_pc)
        goal_pc = self.format_pc(self.goal_pc)
        # print("===========", current_pc, goal_pc)
        with torch.no_grad():
            desired_position = self.model(current_pc.unsqueeze(0), goal_pc.unsqueeze(0))[0].detach().numpy()*(0.001)  
        print(" === desired_position: ", desired_position)
        # self.count += 1
        # if self.count == 1:
        #     desired_position = [0.03,0.03,0.05]
        # elif self.count == 2:
        #     desired_position = [0.05,0.03,0.035]      
        # elif self.count == 3:
        #     desired_position = [0.07,-0.0,-0.00]             

        return desired_position

    def down_sampling(self, pc):
        farthest_indices,_ = farthest_point_sampling(pc, 1024)
        pc = pc[farthest_indices.squeeze()]  
        return pc

    def format_pc(self, pc):
        # farthest_indices,_ = farthest_point_sampling(pc, 1024)
        # pc = pc[farthest_indices.squeeze()]    
        pc = torch.from_numpy(pc).permute(1,0).float()    
        return pc 

         

def deformernet_client():
    rospy.loginfo('Waiting for service get_dn_baxter.')
    rospy.wait_for_service('get_dn_baxter')
    rospy.loginfo('Calling service get_dn_baxter.')
    try:
        dn_baxter_proxy = rospy.ServiceProxy('get_dn_baxter', DNBaxter)
        dn_baxter_request = DNBaxterRequest()
        dn_baxter_response = dn_baxter_proxy(dn_baxter_request) 

    except (rospy.ServiceException, rospy.ROSException), e: #(rospy.ServiceException, e): #except (rospy.ServiceException, e):
        rospy.loginfo('Service get_dn_baxter call failed: %s' % (e,))
    rospy.loginfo('Service get_dn_baxter is executed.')
    return dn_baxter_response.desired_pos

