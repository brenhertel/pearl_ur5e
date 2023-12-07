#!/usr/bin/env python

#https://dof.robotiq.com/discussion/1649/control-gripper-via-ur-controller-client-interface
#got from link above
# Echo client program
import socket
import time
import binascii
from ast import literal_eval
import rospy
from std_msgs.msg import Int32

s = None
c = None
s_self = None



def set_position(data, c):
    #bin_data1 = '{0:32b}'.format(data.data)
    #bin_data2 = format(data.data, "08b")
    rospy.logwarn('data recieved ' + str(data.data))
    #print(data.data)
    #print(bin_data1)
    #print(bin_data2)
    #print(chr(data.data).encode())
    #print(str(data.data).encode())
    c.send(chr(data.data).encode())

def listener(c):
    rospy.init_node('gripper_pose_sub', anonymous=True)
    print('Connected!')
    try:
        rospy.logwarn('listening to /gripper_sends/position')
        rospy.Subscriber('/gripper_sends/position', Int32, set_position, c)
    
        rospy.spin()
        
        rospy.logerr('Closing Socket Connection')
        s.close()
        c.close()
        s_self.close()
        
    except rospy.ROSInterruptException:
        rospy.logerr('Closing Socket Connection')
        s.close()
        c.close()
        s_self.close()
    except KeyboardInterrupt:
        rospy.logerr('Closing Socket Connection')
        s.close()
        c.close()
        s_self.close()
        
    

if __name__ == '__main__':
    rospy.logwarn("Robot must be in Remote Control mode!")
    HOST = "192.168.50.3" # The UR IP address
    PORT = 30005 # UR primary client
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    
    #HOST_self = "127.0.0.1" # own IP address
    #PORT_self = 80 # connection port
    s_self = socket.socket()
    bound = 0
    while not bound:
    	try:
    	    s_self.bind(('', 32345))
    	    bound = 1
    	except:
    	    rospy.logwarn('Could not bind to port, retrying')
    	    rospy.sleep(0.1)
    	    bound = 0
    		
    s_self.listen(5)
    
    f = open ("/home/pearl/catkin_ws/src/pearl_ur5e/src/scripts/gripper_control/get_pose_from_comp.script", "rb")   #Robotiq Gripper
    #f = open ("setzero.script", "rb")  #Robotiq FT sensor
    
    ln = f.read(1024)
    while (ln):
        s.send(ln)
        ln = f.read(1024)
    
    c = None
        
    while not c:
        print('trying...')
        c, addr = s_self.accept()
    try:
        listener(c)
    except:
        s.close()
        c.close()
        s_self.close()
    


