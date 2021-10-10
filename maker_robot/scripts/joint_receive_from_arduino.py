#!/usr/bin/env python
import rospy

from rospy_tutorials.msg import Floats
from three_dof_planar_manipulator.srv import Floats_array, Floats_arrayResponse, Floats_arrayRequest

p1=0
v1=0

p2=0
v2=0

def right_callback(msg):
    global p1, v1 
    p1  = msg.data[0]
    v1  = msg.data[1]

def left_callback(msg):
    global p2, v2 
    p2  = msg.data[0]
    v2  = msg.data[1]
    

def right_server(req):
    global p1, v1
    res = Floats_arrayResponse() 
    res.res=[p1,v1]
    return res

def left_server(req):
    global p2, v2
    res = Floats_arrayResponse() 
    res.res=[p2,v2]
    return res


rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

rospy.Subscriber("right_ticks", Floats, right_callback, queue_size=10)
rospy.Subscriber("left_ticks", Floats, left_callback, queue_size=10)

rospy.Service('right_joint_states', Floats_array, right_server)
rospy.Service('left_joint_states', Floats_array, left_server)


rospy.spin() 
