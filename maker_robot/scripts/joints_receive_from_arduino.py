#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from three_dof_planar_manipulator.srv import Floats_array, Floats_arrayResponse, Floats_arrayRequest

right_pos=0
right_vel=0

left_pos=0
left_vel=0

def my_callback(msg):
    global right_pos, left_pos , right_vel , left_vel
    right_pos  = msg.data[3]
    right_vel  = msg.data[2]

    left_pos =   msg.data[1]
    left_vel =   msg.data[0] 
    #print "Hello Pos: ", pos, "vel: ", vel



def my_server(req):
    global right_pos, left_pos , right_vel , left_vel
    res = Floats_arrayResponse() 
    res.res=[right_pos, right_vel , left_pos , left_vel ]
    return res


rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

rospy.Subscriber("/joint_states_from_arduino", Floats, my_callback, queue_size=10)
 
rospy.Service('/read_joint_state1', Floats_array, my_server)

rospy.spin() 
