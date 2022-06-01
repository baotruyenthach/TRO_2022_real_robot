#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
import csv
import sys
import roslib.packages as rp
import baxter_interface
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from std_msgs.msg import UInt16
import rospy

pkg_path = rp.get_pkg_dir('baxter_moveit_tutorial')
sys.path.append(pkg_path + '/src')

rospy.init_node('baxter_resolved_rate_controller')
arm = baxter_interface.Limb('left')
robot_description = URDF.from_parameter_server()
base_link = 'right_arm_mount'
end_link = 'right_wrist'
kdl_kin = KDLKinematics(robot_description, base_link, end_link)




# csv file name
filename = "/home/baothach/baxter_test_ws/test.csv" 

# initializing the titles and rows list

datas = []
legends = []
ee_positions = []

# reading csv file
with open(filename, 'r') as csvfile:
    # creating a csv reader object
    csvreader = csv.reader(csvfile)
    
    # # # extracting field names through first row
    # fields = next(csvreader)

    # extracting each data row one by one
    for i, row in enumerate(csvreader):
        if i == 0:
            legends = row[1:9]
        else:
            data = [float(item) for item in row][1:9]
            ee_positions.append(np.asarray(kdl_kin.forward(data[:7]))[:3,3])

            datas.append(data)


joint_upper_lims = np.array([1.70167994, 1.047, 3.05417994, 2.618, 3.059, 2.094, 3.059])
joint_low_lims = np.array([-1.70167994, -2.147, -3.05417994, -0.05, -3.059, -1.57079633,-3.059])

datas = np.array(datas)
print(datas.shape)
ee_positions = np.array(ee_positions)
print(ee_positions.shape)

plt.figure()
for i in range(0,7):
    plt.plot(datas[:,i]-joint_low_lims[i])

plt.figure()
for i in range(3):
    plt.plot(ee_positions[:,i])


# plt.legend(range(1,8))
plt.legend(legends)
# # plt.ylim((-0.1,1.5))
# plt.xlabel("z")
# plt.ylabel("p(z)")
plt.show()
