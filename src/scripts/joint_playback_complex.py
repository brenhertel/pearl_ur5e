#!/usr/bin/env python3

# based on example from: https://github.com/harvard-microrobotics/simple_ur_move/blob/devel-noetic/scripts/run_joint_trajectory.py

# Import ROS stuff
import rospy
import rospkg
import os
import numpy as np

from simple_ur_move.joint_trajectory_handler import JointTrajectoryHandler


filepath_config = os.path.join(rospkg.RosPack().get_path('simple_ur_move'), 'config')

#joint_traj = np.loadtxt('failmap_pouring_jointtraj3.txt')
joint_traj = np.loadtxt('imit_dom2000.txt')
#joint_traj = joint_traj[:, 1:]
timings = False
if timings:
    temp = np.copy(joint_traj[:, 1])
    joint_traj[:, 1] = joint_traj[:, 3]
    joint_traj[:, 3] = temp
else:
    temp = np.copy(joint_traj[:, 0])
    joint_traj[:, 0] = joint_traj[:, 2]
    joint_traj[:, 2] = temp

def create_traj(traj, timings=False, max_time=5):
    points = []
    for i in range(len(traj)):
        if timings:
            point = {'time':traj[i, 0], 'positions': traj[i, 1:]}
        else:
            point = {'time':(max_time*i / len(traj)), 'positions': traj[i, :]}
        points.append(point)
    return points


if __name__ == '__main__':
    rospy.init_node('ur5e_joint_traj_runner', anonymous=True)
    
    # Create a trajectory handler
    try:
        traj_handler = JointTrajectoryHandler(name="", controller="scaled_pos_joint_traj_controller", debug=False)
        traj_file="test_traj_joint.yaml"
        traj_handler.load_config(filename=traj_file, directory=filepath_config)
        traj_handler.units['orientation'] = 'radians'
    except:
        raise

    # Run the trajectory
    try:
        traj = create_traj(joint_traj, timings)
        print(traj)
        traj_handler.set_initialize_time(3.0)
        
        print('Starting position: ', traj[0]['positions'])
        print('Press [Enter] to go to starting position')
        input()
        point = {'positions':traj[0]['positions']}
        traj_handler.go_to_point(point)

        print('Press [Enter] to execute trajectory')
        input()
        traj_handler.run_trajectory(trajectory=traj, blocking=True)
        traj_handler.shutdown()

    except KeyboardInterrupt:
        print("ur5e_joint_traj_runner: Shutting Down")
        traj_handler.shutdown()

    except rospy.ROSInterruptException:
        print("ur5e_joint_traj_runner: Shutting Down")
        traj_handler.shutdown()

   



        
