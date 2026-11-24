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
from tf.msg import tfMessage
from std_msgs.msg import Int32
from moveit_commander.conversions import pose_to_list
import h5py
import roslib
import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

fname1 = 'h5_files/recorded_demo 2026-05-25 13:58:40.h5'
idxs1 = [(0, 13161), (13161, 18506), (18506, 37410), (37410, 45785), (45785, 62217), (62217, 69795), (69795, 79606), (79606, 91810)]

fname1 = 'h5_files/recorded_demo 2026-05-26 16:02:15.h5'
idxs1 = [(0, 5863), (5863, 9152), (9152, 20449), (20449, 24667), (24667, 37036), (37036, 43757), (43757, 68853), (68853, 85883)]

fname1 = 'h5_files/recorded_demo 2026-06-02 15:51:00.h5'
idxs1 = [(0, 3666), (3666, 7576), (7576, 13848), (13848, 17677), (17677, 23053), (23053, 27452), (27452, 30628), (30628, 31850), (31850, 39263), (39263, 44558), (44558, 70136), (70136, 72417), (72417, 76001), (76001, 82029), (82029, 97840)]

#fname1 = 'h5_files/recorded_demo 2026-06-04 15:02:03.h5'
#idxs1 = [(0, 8882), (8882, 14392), (14392, 19408), (19408, 22204), (22204, 27071), (27071, 32484), (32484, 35280), (35280, 39063), (39063, 46218), (46218, 49919), (49919, 75002), (75002, 77798), (77798, 81169), (81169, 87831), (87831, 98780)]

fname2 = 'h5_files/recorded_demo 2026-05-25 14:06:04.h5'
idxs2 = [(0, 2772), (14884, 19255), (19255, 33341), (33341, 38285), (2772, 14884), (38285, 43905), (43905, 69228), (69228, 84663)]


fname_long = 'h5_files/recorded_demo 2026-01-29 16:52:33.h5'
idxs1 = [4425, 6905, 10363, 11844, 12839, 16307, 17790, 19282, 20275, 22752, 24248, 26739, 28225, 30705, 31691, 37158, 40607, 47053, 49028]

#fname_towers = 'h5_files/recorded_demo 2026-07-20 14:30:14.h5'
#idxs1 = [4111, 14785, 25892, 37648, 48538, 57698, 66136, 83229]

#fname_drawer = 'h5_files/recorded_demo 2026-07-20 16:50:15.h5'
#idxs1 = [4200, 7800, 12500, 17000]

#fname_cups = 'h5_files/recorded_demo 2026-07-09 15:45:01.h5'
#idxs1 = [0, 7701, 14301, 21059, 29938, 32687]

#fname_microwave = 'h5_files/recorded_demo 2026-07-09 15:22:44.h5'
#idxs1 = [0, 4988, 8850, 12590, 19308, 20894, 27611]

DEMO_NAME = fname_long
REPRO_NAME = 'longtask_segment0.txt'
START_IDX = idxs1[0]
END_IDX = idxs1[1]

X_OFFSET = 0.24
Y_OFFSET = 0.01

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
    print(sys.argv)
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
    
    # reduce velocirty scaling
    move_group.set_max_velocity_scaling_factor(0.25)

    # Misc variables
    self.gripper_publisher = rospy.Publisher('/gripper_sends/position', Int32, queue_size=20)
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
    joint_goal[0] = js_array[i][2]
    joint_goal[1] = js_array[i][1]
    joint_goal[2] = js_array[i][0]
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
    wpose.position.x = -pos_data[i][0] + X_OFFSET
    wpose.position.y = -pos_data[i][1] + Y_OFFSET
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
    
    #js_data = np.loadtxt('path_joint.txt')

    #### move to starting position ####
    self.goto_joint_state(js_data, 0)
    print("Press 'Enter' to start")
    input()
    
    for i in range(1, len(js_data), 25):
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
    
    repro_traj = np.loadtxt(REPRO_NAME, delimiter=',')
    print('start')
    print(repro_traj[0, :])
    print('end')
    print(repro_traj[-1, :])
    (n_pts, n_dims) = np.shape(repro_traj)
    
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
    
    start_idx = START_IDX
    end_idx = END_IDX
    
    # get current segment data
    pos_data = pos_data[start_idx:end_idx]
    rot_data = rot_data[start_idx:end_idx]
    js_data = js_data[start_idx:end_idx]
    
    repro_traj = np.loadtxt(REPRO_NAME)
    print('start')
    print(repro_traj[0, :])
    print('end')
    print(repro_traj[-1, :])
    (n_pts, n_dims) = np.shape(repro_traj)
    
    #### move to starting position ####
    if start_idx == idxs1[0]:
        print("Press 'Enter' to start")
        input()
        print('Going to Joint State: ' + str(js_data[0]))
        self.goto_joint_state(js_data, 0)
    else:
        self.goto_xyz(repro_traj, rot_data)
    
    #repro_traj[:, 0] -= 0.23
    
        
    #print("Press 'Enter' to move to starting position from xyz coords")
    #input()
    self.gripper_publisher.publish(int(repro_traj[0, 3]* 1.1 * 100))

    
    (n_pts_og, _) = np.shape(rot_data)
    og_moments = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    moments = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    #og_moments = np.linspace(0, 1, 64)
    #moments = np.linspace(0, 1, 64)
    true_inds = [int(og_moments[i] * (n_pts_og - 1)) for i in range(len(og_moments))]
    key_inds = [int(moments[i] * (n_pts - 1)) for i in range(len(moments))]
    key_rots = R.from_quat([ [rot_data[ind][0], rot_data[ind][1], rot_data[ind][2], rot_data[ind][3]] for ind in true_inds])
    print(key_rots.as_quat())
    print(true_inds)
    print(key_inds)
    
    '''
    rots = np.loadtxt('repros/quatsX.txt')
    key_inds = [int(i) for i in np.linspace(0, len(repro_traj), len(rots))]
    key_rots = R.from_quat([ [rots[ind][0], rots[ind][1], rots[ind][2], rots[ind][3]] for ind in range(len(rots))])
    '''
    slerp = Slerp(key_inds, key_rots)
    
    #print('Press enter to continue')
    #input()
    print('Planning')
    
    #self.move_group.set_planning_time(5)
    #self.move_group.set_num_planning_attempts(10)
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    waypoints.append(copy.deepcopy(wpose))
    for i in range(1, n_pts):
      if True: #np.linalg.norm(repro_traj[i][:3] - np.array([ -waypoints[-1].position.x, -waypoints[-1].position.y, waypoints[-1].position.z])) >= 0.001: 
        wpose.position.x = -repro_traj[i][0] + X_OFFSET #/tf and rviz have x and y opposite signs
        wpose.position.y = -repro_traj[i][1] + Y_OFFSET
        wpose.position.z = repro_traj[i][2]
        
        if (REPRO_NAME == 'longtask_segment0.txt'):
            wpose.position.x -= (((n_pts - i) / n_pts)**2 * X_OFFSET)
            
        if (REPRO_NAME == 'longtask_segment17.txt'):
            wpose.position.x -= ((i / n_pts) * X_OFFSET)
        
        #print(repro_traj[i][0], repro_traj[i][2], repro_traj[i][3])
        
        cur_R = slerp(np.array([i]))
        cur_quats = cur_R.as_quat()
        #print(cur_quats)
        wpose.orientation.x = -cur_quats[0][1]
        wpose.orientation.y = cur_quats[0][0]
        wpose.orientation.z = cur_quats[0][3]
        wpose.orientation.w = -cur_quats[0][2]
        
        '''
        new_quats = np.array([-cur_quats[0][1] - 0.2, cur_quats[0][0] - 0.1, cur_quats[0][3] - 0.1, -cur_quats[0][2]])
        new_quats = new_quats / np.linalg.norm(new_quats)
        wpose.orientation.x = new_quats[0]
        wpose.orientation.y = new_quats[1]
        wpose.orientation.z = new_quats[2]
        wpose.orientation.w = new_quats[3]
        '''
        waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.002,       # eef_step
                                       0.0)       # jump_threshold
    print("Planning for " + str(fraction * 100) + "% of trajectory acheived!")
    if fraction < 0.9:
    	print("Not enough planned! Exiting...")
    	exit()
    if plan.joint_trajectory.points[1].time_from_start.secs + plan.joint_trajectory.points[1].time_from_start.nsecs * 1e-9 <= plan.joint_trajectory.points[0].time_from_start.secs + plan.joint_trajectory.points[0].time_from_start.nsecs * 1e-9 :
        print('(1) Trajectory will fail because of time ordering of points! fixing...')
        plan.joint_trajectory.points[1].time_from_start.nsecs = (plan.joint_trajectory.points[0].time_from_start.nsecs + plan.joint_trajectory.points[2].time_from_start.nsecs) // 2 
    for i in range(1, len(plan.joint_trajectory.points)-1):
        if plan.joint_trajectory.points[i+1].time_from_start.secs + plan.joint_trajectory.points[i+1].time_from_start.nsecs * 1e-9 <= plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[i].time_from_start.nsecs * 1e-9 :
            print(i-1, i, i+1)
            print(plan.joint_trajectory.points[i-1].time_from_start.secs + plan.joint_trajectory.points[i-1].time_from_start.nsecs * 1e-9)
            print(plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[i].time_from_start.nsecs * 1e-9)
            print(plan.joint_trajectory.points[i+1].time_from_start.secs + plan.joint_trajectory.points[i+1].time_from_start.nsecs * 1e-9)
            print('(2) Trajectory will fail because of time ordering of points! fixing...')
            #exit()
            plan.joint_trajectory.points[i].time_from_start.nsecs = (plan.joint_trajectory.points[i-1].time_from_start.nsecs + plan.joint_trajectory.points[i+1].time_from_start.nsecs) // 2 
            
    return plan

  def display_trajectory(self, plan):
    #ask rviz to display the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory)

  def execute_plan(self, plan):
    #execute given plan
    self.move_group.execute(plan, wait=True)
    
  def execute_plan_gripper(self, plan):
    #execute given plan
    repro_traj = np.loadtxt(REPRO_NAME)
    #repro_traj[:, 0] = repro_traj[:, 0] - 0.2
    gr_points = repro_traj[:, 3]
    
    #print(gr_points)
    #print(plan)
    end_time = plan.joint_trajectory.points[-1].time_from_start.secs
    #print(end_time)
    self.move_group.execute(plan, wait=False)
    for i in range(len(gr_points)):
        self.gripper_publisher.publish(int(gr_points[i] * 1.1 * 100))
        rospy.sleep(end_time / len(gr_points))
    
  def execute_plan_gripper2(self, plan):
    #execute given plan
    repro_traj = np.loadtxt(REPRO_NAME)
    repro_traj[:, 0] -= X_OFFSET
    repro_traj[:, 1] -= Y_OFFSET
    gr_points = repro_traj[:, 3]
    
    self.move_group.execute(plan, wait=False)
    i = 0
    check = 0
    while i < len(gr_points):
        pos_msg = rospy.wait_for_message("/tf", tfMessage, timeout=10)
        if pos_msg.transforms[0].child_frame_id == 'tool0_controller':
            pos = np.array([pos_msg.transforms[0].transform.translation.x, pos_msg.transforms[0].transform.translation.y, pos_msg.transforms[0].transform.translation.z])
            #print(pos, traj_data[i], np.linalg.norm(pos - traj_data[i]))
            if np.linalg.norm(pos - repro_traj[i, :3]) < 0.01:
                #print(i, gr_points[i])
                self.gripper_publisher.publish(min(100, max(1, int(abs(gr_points[i+2]) * 1.2 * 100))))
                i = i + 5
                check = 0
            elif np.linalg.norm(pos - repro_traj[-1, :3]) < 0.01:
                if check == 0:
                    print('Execution finished, but gripper did not. Check Once...')
                    check += 1
                if check != 0:
                    self.gripper_publisher.publish(min(100, max(1, int(abs(gr_points[-1]) * 1. * 100))))
                    print('Execution finished, but gripper did not. Check Twice... Exiting')
                    return
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
    box_pose.pose.position.z = -0.11
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
    box_pose.pose.position.y = -0.3 # next to the robot
    self.box_name2 = "wall"
    self.scene.add_box(self.box_name2, box_pose, size=(10, 0.02, 10))
    return self.wait_for_state_update(self.box_name2, box_is_known=True, timeout=timeout)
 
  def remove_workspace(self, timeout=4):
    #remove each object from the planning scene, waiting for scene to update before moving on
    self.scene.remove_world_object(self.box_name1)
    self.wait_for_state_update(self.box_name1, box_is_attached=False, box_is_known=False, timeout=timeout)
    self.scene.remove_world_object(self.box_name2)
    return self.wait_for_state_update(self.box_name2, box_is_attached=False, box_is_known=False, timeout=timeout) 

def execute_succession():
  try:
    print("Playing back a demo")
    print("Press Ctrl-D to exit at any time")
    print("Press 'Enter' to begin")
    input()
    ur5e_arm = MoveGroupPythonInterface()
    #table and wall have to be added in separately--for some reason adding them together didn't work
    ur5e_arm.add_table()
    ur5e_arm.add_wall()
    
    #txt_names = ['coffee_segment0.txt', 'coffee_segment1.txt', 'coffee_segment2.txt', 'coffee_segment3.txt', 'coffee_segment4.txt', 'coffee_segment5.txt', 'coffee_segment6.txt', 'coffee_segment7.txt', 'coffee_segment8.txt', 'coffee_segment9.txt', 'coffee_segment10.txt', 'coffee_segment11.txt', 'coffee_segment12.txt', 'coffee_segment13.txt', 'coffee_segment14.txt']
    #txt_names = ['coffee_segment8.txt', 'coffee_segment9.txt', 'coffee_segment10.txt', 'coffee_segment11.txt', 'coffee_segment12.txt', 'coffee_segment13.txt', 'coffee_segment14.txt']
    #txt_names = ['hanoi_segment20.txt', 'hanoi_segment21.txt', 'hanoi_segment22.txt', 'hanoi_segment23.txt', 'hanoi_segment24.txt', 'hanoi_segment25.txt', 'hanoi_segment26.txt']
    #txt_names = ['drawer2k_segment0.txt', 'drawer2k_segment1.txt', 'drawer2k_segment2.txt',]
    #txt_names = ['cups_segment0.txt', 'cups_segment1.txt', 'cups_segment2.txt', 'cups_segment3.txt', 'cups_segment4.txt']
    #txt_names = ['microwave_segment0.txt', 'microwave_segment1.txt', 'microwave_segment2.txt', 'microwave_segment3.txt', 'microwave_segment4.txt', 'microwave_segment5.txt']
    #txt_names = ['microwave_q_segment0.txt', 'microwave_q_segment1.txt', 'microwave_q_segment2.txt', 'microwave_q_segment3.txt', 'microwave_q_segment4.txt', 'microwave_q_segment5.txt']
    #txt_names = ['microwave_segment2.txt', 'microwave_segment3.txt', 'microwave_segment4.txt', 'microwave_segment5.txt']
    txt_names = ['longtask_segment0.txt', 'longtask_segment1.txt','longtask_segment2.txt', 'longtask_segment3.txt','longtask_segment4.txt', 'longtask_segment5.txt', 'longtask_segment6.txt','longtask_segment7.txt', 'longtask_segment8.txt','longtask_segment9.txt', 'longtask_segment10.txt', 'longtask_segment11.txt', 'longtask_segment12.txt', 'longtask_segment13.txt', 'longtask_segment14.txt', 'longtask_segment15.txt', 'longtask_segment16.txt','longtask_segment17.txt']
    #txt_names = ['longtask_segment9.txt', 'longtask_segment10.txt', 'longtask_segment11.txt', 'longtask_segment12.txt', 'longtask_segment13.txt', 'longtask_segment14.txt', 'longtask_segment15.txt', 'longtask_segment16.txt','longtask_segment17.txt']
    idxs = idxs1#[9:]
    #idxs = [ 24248, 26739, 28225, 30705, 31691, 37158, 40607, 47053, 49028]
    for i in range(len(txt_names)):
        global REPRO_NAME, START_IDX, END_IDX
        REPRO_NAME = txt_names[i]
        START_IDX = idxs[i]
        END_IDX = idxs[i+1]
        print(txt_names[i], idxs[i], idxs[i+1])
        print("Press 'Enter' to plan trajectory")
        input()
        plan = ur5e_arm.plan_cartesian_path()
        ur5e_arm.display_trajectory(plan)
        ur5e_arm.execute_plan_gripper2(plan)
    
    print("Execution complete")
    #print("Press 'Enter' to exit'")
    #input()
    ur5e_arm.remove_workspace()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


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
    #ur5e_arm.execute_joint_path()
    
    #print("Press 'Enter' to execute")
    #input()
    #ur5e_arm.exec_cartesian_path()
    
    print("Press 'Enter' to plan trajectory")
    input()
    plan = ur5e_arm.plan_cartesian_path()
    ur5e_arm.display_trajectory(plan)
    print("Press 'Enter' to execute trajectory")
    input()
    #ur5e_arm.execute_plan(plan)
    ur5e_arm.execute_plan_gripper2(plan)
    
    print("Execution complete")
    #print("Press 'Enter' to exit'")
    #input()
    ur5e_arm.remove_workspace()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  execute_succession()
  #main()

