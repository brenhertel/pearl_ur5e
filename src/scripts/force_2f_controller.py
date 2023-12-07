import numpy as np
import matplotlib.pyplot as plt
import h5py
import rospy

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
        
        
def main():
    rospy.init_node('gripper_2f_force_controller', anonymous=True)
    
    fname = 'h5_files/recorded_demo 2023-11-28 12:20:30.h5'
    
    FC = ForceController(fname)
    
    
    

if __name__ == '__main__':
    main()
