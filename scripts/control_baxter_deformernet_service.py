#!/usr/bin/env python3

import numpy as np
import rospy
from deformernet_utils import *
import timeit
import open3d
import roslib
# import sys
# import roslib.packages as rp
# pkg_path = rp.get_pkg_dir('deformernet_real_robot')
# sys.path.append(pkg_path + '/src')
from baxter_moveit_tutorial.srv import *
import pickle

dfnet = DeformerNetBaxter(prob_type="plane", constrain_plane=[1,1,0,0.04])  # uncomment if use goal plane

# pcd_goal = open3d.io.read_point_cloud("/home/baothach/shape_servo_data/real_robot/goal_data/video/real_pc_up_1.pcd")
# pcd_goal = open3d.io.read_point_cloud("/home/baothach/shape_servo_data/real_robot/goal_data/video/sim_right_1.pcd")
# goal_pc = np.asarray(pcd_goal.points)  
# simulated_goal_path = "/home/baothach/shape_servo_data/real_robot/goal_data/simulated_box"
# with open(os.path.join(simulated_goal_path, f"sample_{0}.pickle"), 'rb') as handle:
#     data = pickle.load(handle)         
#     goal_pc = data["partial pcs"][1]
# dfnet = DeformerNetBaxter(goal_pc=goal_pc, model_type=1)

class DeformerNetBaxterControl():


    def __init__(self):
        rospy.init_node('DeformerNet_Baxter_server')
        pcd_goal = open3d.io.read_point_cloud("/home/baothach/shape_servo_data/real_robot/goal_data/pc_right_2.pcd")  # uncomment if use goal point cloud
        self.goal_pc = np.asarray(pcd_goal.points)    # uncomment if use goal point cloud      
        

    def create_dn_baxter_server(self):
        '''
        Create service to get desired pos from DeformerNet.
        '''
        rospy.Service('get_dn_baxter', DNBaxter,
                      self.handle_get_deformernet)
        rospy.loginfo('Service get_dn_baxter:')
        rospy.loginfo('Ready to get info from get_dn_baxter.')        

    def handle_get_deformernet(self, req):        
        
        response = DNBaxterResponse()

        dfnet = DeformerNetBaxter(goal_pc=self.goal_pc)  # uncomment if use goal point cloud
        # dfnet = DeformerNetBaxter(prob_type="plane", constrain_plane=[0,1,0,-0.05])
        # success = dfnet.get_current_pc(filter_by_color=True, intensity_thres=0.4, visualization=True, min_num_points=700)
        success = False
        while not success:
            success = dfnet.get_current_pc(filter_by_color=True, intensity_thres=0.6, visualization=False, min_num_points=700)
            # return response

        desired_position = dfnet.run_deformernet()              
        response.desired_pos = list(desired_position)

        return response
        


if __name__ == '__main__':
    dn_baxter_service = DeformerNetBaxterControl()
    dn_baxter_service.create_dn_baxter_server()
    rospy.spin()




