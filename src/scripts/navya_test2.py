#!/usr/bin/env python3

import rospy 
from collections import namedtuple
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header 
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def extract_xyz_val(cloud_msg, row, col): 
	points = pc2.read_points_list(cloud_msg)
	width = cloud_msg.width 
	index = row* width + col 
	
	point = points[index] 
	x = point.x
	y = point.y 
	z = point.z 
	return x, y, z 
	
def callback(msg): 
	row = 360
	col = 500
	x,y,z = extract_xyz_val(msg, row, col)
	
	t = 0
	rot_matrix_y = np.array(([[1,           0,        0],
                                 [0,    np.cos(t),  -np.sin(t)],
                                 [0,    np.sin(t),   np.cos(t)]]))

	t = np.pi
	rot_matrix_y = np.array(([[np.cos(t),  0, np.sin(t)], [0,          1,         0],   [-np.sin(t), 0, np.cos(t)]]))
    
    t = -np.pi / 2
    rot_matrix_z = np.array(([[np.cos(t), -np.sin(t), 0], [np.sin(t),    np.cos(t), 0],  [0,            0,         1]]))
                                
    rot_matrix_final = rot_matrix_x @ rot_matrix_y @ rot_matrix_z
      
    transformation_mat = np.zeros(4,4)

	transform_mat[:3, :3] = rot_matrix_final
	transform_mat[0,   3] = 0.47
	transform_mat[1,   3] = 0.44
	transform_mat[2,   3] = 1.17
	transform_mat[3,   3] = 1

	 
	pos_vector = np.array([[x], [y], [z], [1])
	transform_pos = transform_mat @ pos_vector
	x_new = transform_pos[0]
	y_new = transform_pos[1]
	z_new = transform_pos[2]
	print('Pixel position {},{} : x = {}, y={}, z{}' .format(row, col, x_new, y_new, z_new))

def transform_points(x, y, z)

	t = 0
	rot_matrix_y = np.array(([[1,           0,        0],
                                 [0,    np.cos(t),  -np.sin(t)],
                                 [0,    np.sin(t),   np.cos(t)]]))

	t = np.pi
	rot_matrix_y = np.array(([[np.cos(t), 0, np.sin(t)],
                                [0,           1,         0],
                                [-np.sin(t),  0, np.cos(t)]]))
        
    t = -(np.pi/2)
    rot_matrix_z = np.array(([[np.cos(t), -np.sin(t), 0],
                              [np.sin(t),    np.cos(t), 0],
                              [0,            0,         1]]))
                                
    rot_matrix_final = rot_matrix_x @ rot_matrix_y @ rot_matrix_z
        
    transformation_mat = np.zeros(4,4)

	transform_mat[:3, :3] = rot_matrix_final
	transform_mat[0,   3] = 0.47
	transform_mat[1,   3] = 0.44
	transform_mat[2,   3] = 1.17
	transform_mat[3,   3] = 1

	 
	pos_vector = np.array([[x], [y], [z], [1])
	transform_pos = transform_mat @ pos_vector
	x_new = transform_pos[0]
	y_new = transform_pos[1]
	z_new = transform_pos[2]
	print('Transform position : x = {}, y={}, z{}' .format(x_new, y_new, z_new))
	return (x_new, y_new, z_new)

if __name__=='__main__':
	rospy.init_node('xyz_coordinate_node')
	rospy.Subscriber('/camera/depth/color/points', PointCloud2, None) 
	rospy.spin() 
	#print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

	
