import numpy as np
import matplotlib.pyplot as plt
import h5py
import rospy

import sys
import copy
import moveit_commander
from math import pi
from moveit_commander.conversions import pose_to_list
import roslib

import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int32
from pearl_ur5e.msg import gripper_pos

from plot_force_demo_2f import read_data

def calc_moving_avg(x, window_size, step_size):
    y = np.arange(window_size//2, len(x) - window_size//2, step_size)
    new_x = np.zeros(np.shape(y))
    for i in range(len(new_x)):
        new_x[i] = np.mean(x[y[i] - window_size//2 : y[i] + window_size//2])
    return new_x, y

class ForceController(object):

    def __init__(self, fname):
        joint_data, tf_data, wrench_data, gripper_data = read_data(fname)
        
        time = gripper_data[0][:, 0] + gripper_data[0][:, 1] * (10.0**-9)
        gpos = gripper_data[1]
        ftime = gripper_data[2][:, 0] + gripper_data[2][:, 1] * (10.0**-9)
        gforce = gripper_data[3]
        
        self.force_avg, inds = calc_moving_avg(gforce, 128, 32)
        self.time_checks = ftime[inds] - ftime[0]
        
        self.t0 = None
        
        self.cur_pos = int(gpos[0])
        
        self.pub = rospy.Publisher('/gripper_sends/position', Int32, queue_size=1)
        rospy.sleep(0.1)
        self.new_msg = Int32()
        self.new_msg.data = self.cur_pos
        self.pub.publish(self.new_msg)
        
        try:
            rospy.Subscriber('/gripper_sensors', gripper_pos, self.check_force, queue_size=1)
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logwarn('Shutting down')
            self.new_msg.data = int(gpos[-1])
            self.pub.publish(self.new_msg)
        except KeyboardInterrupt:
            rospy.logwarn('Shutting down')
            self.new_msg.data = int(gpos[-1])
            self.pub.publish(self.new_msg)
        
    def check_force(self, msg):
        if self.t0 is None:
            self.t0 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10.0**-9)
        else:
            cur_time = (msg.header.stamp.secs + msg.header.stamp.nsecs * (10.0**-9)) - self.t0
            if cur_time <= self.time_checks[-1]:
                inds_passed = cur_time > self.time_checks
                gripper_forces = self.force_avg[inds_passed]
                if len(gripper_forces) > 0:
                    tgt_force = gripper_forces[-1]
                    cur_force = msg.gripper_pos
                    print(cur_time, cur_force, tgt_force, self.cur_pos)
                    if cur_force*1.1 < tgt_force:
                        if self.cur_pos < 95:
                            self.new_msg.data = self.increase_pos(cur_force, tgt_force)
                            self.pub.publish(self.new_msg)
                            rospy.sleep(0.1)
                    if cur_force > tgt_force*1.1:
                        if self.cur_pos > 5:
                            self.new_msg.data = self.decrease_pos(cur_force, tgt_force)
                            self.pub.publish(self.new_msg)
                            rospy.sleep(0.1)
    
    def increase_pos(self, cur_force, tgt_force):
        self.cur_pos = self.cur_pos + 1 + int(tgt_force // cur_force)
        if self.cur_pos > 95:
            self.cur_pos = 95
        return self.cur_pos

    
    def decrease_pos(self, cur_force, tgt_force):
        self.cur_pos = self.cur_pos - 1 - int(cur_force // tgt_force)
        if self.cur_pos < 5:
            self.cur_pos = 5
        return self.cur_pos
        
        

#not actually sure why this is here or why its necessary, but I suppose its a good way to test tolerances
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

#How the robot is understood and controlled
class MoveGroupPythonInterface(object):
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        #the moveit_commander is what is responsible for sending info the moveit controllers
        moveit_commander.roscpp_initialize(sys.argv)
        #Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()
        #Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        #Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints), which in our moveit setup is named 'manipulator'
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        #Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
     
        #Get all the info which is carried with the interface object
        #We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        #print "Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print  End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        #print  Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.box_name1 = ''
        self.box_name2 = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def goto_joint_state(self, js_array, i=0):
        # To start the playback of the demo, go to the initial demo position, which can be interpreted as the 0th set of joint states
        # I use joint states instead of cartesians because cartesians will fail if the current state is too far away from the goal state, whereas joint states will simply execute
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = js_array[i][0]
        joint_goal[1] = js_array[i][1]
        joint_goal[2] = js_array[i][2]
        joint_goal[3] = js_array[i][3]
        joint_goal[4] = js_array[i][4]
        joint_goal[5] = js_array[i][5]
        # go to the initial position
        # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
  
    def execute_joint_path(self, fname, scale=1):
        #start planning demo playback by reading data from the demo.h5 file
        
        hf = h5py.File(fname, 'r')
        #navigate to necessary data and store in numpy arrays
        tf_info = hf.get('transform_info')
        js_info = hf.get('joint_state_info')
        pos_data = tf_info.get('transform_positions')
        rot_data = tf_info.get('transform_orientations')
        pos_data = np.array(pos_data)
        rot_data = np.array(rot_data)
        js_data = js_info.get('joint_positions')
        js_data = np.array(js_data)
        #close out file
        hf.close()

        #### move to starting position ####
        self.goto_joint_state(js_data, 0)
        #print("Press 'Enter' to start")
        #input()
        
        for i in range(1, len(js_data), 50):
            self.goto_joint_state(js_data, i)
     
        return 

    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        #either this times out and returns false or the object is found within the planning scene and returns true
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False


    def add_table(self, timeout=4):
        #define a box for the table below the robot
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        #box origin (default = {0, 0, 0, 0, 0, 0, 0})
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.07
        self.box_name1 = "table"
        #add box to planning scene and specify dimensions
        self.scene.add_box(self.box_name1, box_pose, size=(10, 10, 0.1))
        #wait for the box to be added in or to timeout
        return self.wait_for_state_update(self.box_name1, box_is_known=True, timeout=timeout)

    def add_wall(self, timeout=4):
        #Same as above with different dimensions
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.y = -0.15 # next to the robot
        self.box_name2 = "wall"
        self.scene.add_box(self.box_name2, box_pose, size=(10, 0.02, 10))
        return self.wait_for_state_update(self.box_name2, box_is_known=True, timeout=timeout)
 
    def remove_workspace(self, timeout=4):
        #remove each object from the planning scene, waiting for scene to update before moving on
        self.scene.remove_world_object(self.box_name1)
        self.wait_for_state_update(self.box_name1, box_is_attached=False, box_is_known=False, timeout=timeout)
        self.scene.remove_world_object(self.box_name2)
        return self.wait_for_state_update(self.box_name2, box_is_attached=False, box_is_known=False, timeout=timeout) 


def main():
    try:
        rospy.init_node('gripper_2f_force_playback', anonymous=True)
        print("Playing back a demo")
        print("Press Ctrl-D to exit at any time")
        print("Press 'Enter' to begin")
        input()
        ur5e_arm = MoveGroupPythonInterface()
        #table and wall have to be added in separately--for some reason adding them together didn't work
        print("Press 'Enter' to add in obstacles")
        input()
        ur5e_arm.add_table()
        ur5e_arm.add_wall()
        print("Press 'Enter' to execute recorded joint trajectory")
        input()
        fname = 'h5_files/recorded_demo 2023-11-28 12:20:30.h5'
        FC = ForceController(fname)
        ur5e_arm.execute_joint_path(fname)
        print("Execution complete")
        print("Press 'Enter' to exit'")
        input()
        ur5e_arm.remove_workspace()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

    
    
    

if __name__ == '__main__':
    main()
