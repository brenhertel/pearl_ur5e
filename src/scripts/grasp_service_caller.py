#!/usr/bin/env python  
import roslib
import rospy
import rosservice
from dexterous_picking.srv import GetGrasp 


def get_grasp_caller():
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            rospy.wait_for_service('/get_grasp')
            get_grasp = rospy.ServiceProxy('/get_grasp', GetGrasp)
            ret = get_grasp()
            print(ret)
            rate.sleep()
    except KeyboardInterrupt:
        exit()
        return
    except rospy.ROSInterruptException:
        exit()
        return

if __name__ == '__main__':
    rospy.init_node('grasp_service_caller', anonymous=True)
    get_grasp_caller()
