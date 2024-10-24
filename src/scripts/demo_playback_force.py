#!/usr/bin/env python3

#Used https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py as template. See what they do for better explanations. There are parts here--such as setup--which are almost exactly as in that document.

#necessary imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import h5py
import roslib
import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

DEMO_NAME = 'h5_files/recorded_demo 2024-02-15 17:20:05.h5'
REPRO_NAME = 'repros/EFMLObstacle.txt'

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
    #initialize node
    rospy.init_node('demo_xyz_playback', anonymous=True)
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
  
  def goto_xyz(self, pos_data, rot_data, i=0):
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = -pos_data[i][0]
    wpose.position.y = -pos_data[i][1]
    wpose.position.z = pos_data[i][2]
    wpose.orientation.x = -rot_data[i][1]
    wpose.orientation.y = rot_data[i][0]
    wpose.orientation.z = rot_data[i][3]
    wpose.orientation.w = -rot_data[i][2]
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (start_plan, start_fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
    self.move_group.execute(start_plan, wait=True)
  
  def goto_xyz2(self, pos_data, rot_data):
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = -pos_data[0]
    wpose.position.y = -pos_data[1]
    wpose.position.z = pos_data[2]
    wpose.orientation.x = -rot_data[1]
    wpose.orientation.y = rot_data[0]
    wpose.orientation.z = rot_data[3]
    wpose.orientation.w = -rot_data[2]
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (start_plan, start_fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
    self.move_group.execute(start_plan, wait=True)
    
  def execute_joint_path(self, scale=1):
    #start planning demo playback by reading data from the demo.h5 file
    
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    filename = 'h5_files/recorded_demo 2023-07-27 16:10:05.h5'
    #open the file
    hf = h5py.File(filename, 'r')
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
    print("Press 'Enter' to start")
    input()
    
    for i in range(1, len(js_data), 50):
    	self.goto_joint_state(js_data, i)
 
    return 
    
  def exec_cartesian_path(self, scale=1):
    #start planning demo playback by reading data from the demo.h5 file
    
    #ask user for the file which the playback is for
    #filename = raw_input('Enter the filename of the .h5 demo: ')
    filename = DEMO_NAME
    #open the file
    hf = h5py.File(filename, 'r')
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
    print("Press 'Enter' to start")
    input()
    
    repro_traj = np.loadtxt(REPRO_NAME)
    print('start')
    print(repro_traj[0, :])
    print('end')
    print(repro_traj[-1, :])
    (n_pts, n_dims) = np.shape(repro_traj)
    
    
    print("Press 'Enter' to move to starting position from xyz coords")
    input()
    self.goto_xyz(repro_traj, rot_data)

    (n_pts_og, _) = np.shape(rot_data)
    og_moments = [0.0, 0.25, 0.5, 0.75, 1.0]
    moments = [0.0, 0.25, 0.5, 0.75, 1.0]
    true_inds = [int(moments[i] * (n_pts_og - 1)) for i in range(len(og_moments))]
    key_inds = [int(moments[i] * (n_pts - 1)) for i in range(len(moments))]
    key_rots = R.from_quat([ [rot_data[ind][0], rot_data[ind][1], rot_data[ind][2], rot_data[ind][3]] for ind in true_inds])
    #print(key_rots.as_quat())
    #print(true_inds)
    #print(key_inds)
    
    slerp = Slerp(key_inds, key_rots)

    print('Press enter to execute')
    input()
    print('Executing')
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    for i in range(1, n_pts):
      
      cur_R = slerp(np.array([i]))
      cur_quats = cur_R.as_quat()
      self.goto_xyz2(repro_traj[i, :], cur_quats[0])
    return
    
  def plan_cartesian_path(self, scale=1):
    #start planning demo playback by reading data from the demo.h5 file
    
    from force_elmap_maybe import main
    from compile_demos import pull_data_across_users
    repro_traj, force = main() 
    
    data = pull_data_across_users('Plate', 1)
    data_usr1 = data[0]
    [joint_data, tf_data, wrench_data, gripper_data] = data_usr1
    js_data = joint_data[1]
    print(len(tf_data))
    rot_data = tf_data[2]
    print(np.shape(rot_data))
    print(rot_data)
    
    self.gt = gripper_data[2]
    self.force = gripper_data[3]
    
    
    #### move to starting position ####
    self.goto_joint_state(js_data, 0)
    print("Press 'Enter' to start")
    input()
    
    print('start')
    print(repro_traj[0, :])
    print('end')
    print(repro_traj[-1, :])
    (n_pts, n_dims) = np.shape(repro_traj)
    print(np.shape(repro_traj))
    
    
    print("Press 'Enter' to move to starting position from xyz coords")
    input()
    self.goto_xyz(repro_traj, rot_data)

    (n_pts_og, _) = np.shape(rot_data)
    og_moments = [0.0, 0.25, 0.5, 0.75, 1.0]
    moments = [0.0, 0.25, 0.5, 0.75, 1.0]
    true_inds = [int(og_moments[i] * (n_pts_og - 1)) for i in range(len(og_moments))]
    key_inds = [int(moments[i] * (n_pts - 1)) for i in range(len(moments))]
    key_rots = R.from_quat([ [rot_data[ind][0], rot_data[ind][1], rot_data[ind][2], rot_data[ind][3]] for ind in true_inds])
    print(key_rots.as_quat())
    print(true_inds)
    print(key_inds)
    
    slerp = Slerp(key_inds, key_rots)

    #print('Press enter to continue')
    #input()
    print('Planning')
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    for i in range(1, n_pts, 10):
      print(i, repro_traj[i])
      wpose.position.x = -repro_traj[i][0] #/tf and rviz have x and y opposite signs
      wpose.position.y = -repro_traj[i][1] 
      wpose.position.z = repro_traj[i][2] 
      
      cur_R = slerp(np.array([i]))
      cur_quats = cur_R.as_quat()
      print(cur_quats)
      wpose.orientation.x = -cur_quats[0][1]
      wpose.orientation.y = cur_quats[0][0]
      wpose.orientation.z = cur_quats[0][3]
      wpose.orientation.w = -cur_quats[0][2]
      
      waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.005,       # eef_step
                                       0.0)       # jump_threshold
    print("Planning for " + str(fraction * 100) + "% of trajectory acheived!")
    if fraction < 0.4:
    	print("Not enough planned! Exiting...")
    	exit()
    return plan

  def display_trajectory(self, plan):
    #ask rviz to display the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    #execute given plan
    #from force_2f_controller import ForceController
    #fc = ForceController(self.gt, self.force, init_pose=5, final_pose=5)
    self.move_group.execute(plan, wait=True)

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
    box_pose.pose.position.y = -0.2 # next to the robot
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
    print("Playing back a demo")
    print("Press Ctrl-D to exit at any time")
    print("Press 'Enter' to begin")
    input()
    ur5e_arm = MoveGroupPythonInterface()
    #table and wall have to be added in separately--for some reason adding them together didn't work
    ur5e_arm.add_table()
    ur5e_arm.add_wall()
    
    #print("Press 'Enter' to execute")
    #input()
    #ur5e_arm.exec_cartesian_path()
    
    print("Press 'Enter' to plan trajectory")
    input()
    plan = ur5e_arm.plan_cartesian_path()
    ur5e_arm.display_trajectory(plan)
    print("Press 'Enter' to execute trajectory")
    input()
    ur5e_arm.execute_plan(plan)
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

