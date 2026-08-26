#!/usr/bin/env python  
import roslib
import rospy
import tf2_ros
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped

import numpy as np

def broadcast_pose():
    rate = rospy.Rate(50)
    try:
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        publisher = rospy.Publisher('/tf_fixed', TransformStamped, queue_size=20)
        msg = TransformStamped()
        msg.child_frame_id = 'wrist'
        msg.header.frame_id = 'base'
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('base_link',
                                                   'wrist_roll_link',
                                                   rospy.Time())
                #print(trans.transform)
                '''
                other_msg = rospy.wait_for_message('/tf', tfMessage)
                print(trans)
                print(trans.translation.x)
                msg.transform.translation.x = trans.translation.x
                msg.transform.translation.y = trans.translation.y
                msg.transform.translation.z = trans.translation.z
                msg.transform.rotation.x = trans.rotation.x
                msg.transform.rotation.y = trans.rotation.y
                msg.transform.rotation.z = trans.rotation.z
                msg.transform.rotation.w = trans.rotation.w
                msg.header.stamp = other_msg.transforms[0].header.stamp
                '''
                publisher.publish(trans)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
            rate.sleep()
    except KeyboardInterrupt:
        exit()
        return
    except rospy.ROSInterruptException:
        exit()
        return

if __name__ == '__main__':
    rospy.init_node('fetch_broadcaster')
    broadcast_pose()
