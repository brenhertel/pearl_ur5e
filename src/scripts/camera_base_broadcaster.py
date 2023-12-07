#!/usr/bin/env python  
import roslib
import rospy
import tf

import numpy as np

def broadcast_pose():
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            br = tf.TransformBroadcaster()
            br.sendTransform((-0.47, 0.36, 1.11),
                         tf.transformations.quaternion_from_euler(np.pi, np.pi/2, 0),
                         rospy.Time.now(),
                         "camera_link",
                         "base_link")
            rate.sleep()
    except KeyboardInterrupt:
        exit()
        return
    except rospy.ROSInterruptException:
        exit()
        return

if __name__ == '__main__':
    rospy.init_node('camera_base_broadcaster')
    broadcast_pose()
