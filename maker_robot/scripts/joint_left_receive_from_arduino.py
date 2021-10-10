#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from three_dof_planar_manipulator.srv import Floats_array, Floats_arrayResponse, Floats_arrayRequest

p1=0
v1=0


def my_callback(msg):
    global p1, v1 
    p1  = 1# msg.data[0]
    v1  = 2# msg.data[1]
    

def my_server(req):
    global p1, v1
    res = Floats_arrayResponse() 
    res.res=[p1,v1]
    return res


rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

rospy.Subscriber("/left_ticks", Floats, my_callback, queue_size=10)

rospy.Service('/right_joint_states', Floats_array, my_server)


rospy.spin() 
