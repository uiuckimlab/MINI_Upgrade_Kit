#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64

r_forefoot_force = 0
r_sole_force = 0
l_forefoot_force = 0
l_sole_force = 0

def r_forefoot_ft_callback(msg):
    if len(msg.states) == 0:
        return

    global r_forefoot_force 
    avg_force = 0
    for state in msg.states:
        avg_force += state.total_wrench.force.x
    avg_force /= len(msg.states)
    r_forefoot_force = abs(avg_force)

def l_forefoot_ft_callback(msg):
    if len(msg.states) == 0:
        return

    global l_forefoot_force 
    avg_force = 0
    for state in msg.states:
        avg_force += state.total_wrench.force.x
    avg_force /= len(msg.states)
    l_forefoot_force = abs(avg_force)

def r_sole_ft_callback(msg):
    if len(msg.states) == 0:
        return

    global r_sole_force 
    avg_force = 0
    for state in msg.states:
        avg_force += state.total_wrench.force.x
    avg_force /= len(msg.states)
    r_sole_force = abs(avg_force)

def l_sole_ft_callback(msg):
    if len(msg.states) == 0:
        return

    global l_sole_force 
    avg_force = 0
    for state in msg.states:
        avg_force += state.total_wrench.force.x
    avg_force /= len(msg.states)
    l_sole_force = abs(avg_force)

rospy.init_node('ft_contact')

r_forefoot_ft_sensor = rospy.Subscriber("/r_forefoot_link_bumper", ContactsState, r_forefoot_ft_callback)
l_forefoot_ft_sensor = rospy.Subscriber("/l_forefoot_link_bumper", ContactsState, l_forefoot_ft_callback)
r_sole_ft_sensor = rospy.Subscriber("/r_sole_link_bumper", ContactsState, r_sole_ft_callback)
l_sole_ft_sensor = rospy.Subscriber("/l_sole_link_bumper", ContactsState, l_sole_ft_callback)
l_foot_Fz_pub = rospy.Publisher('l_foot_Fz', Float64, queue_size=10)
r_foot_Fz_pub = rospy.Publisher('r_foot_Fz', Float64, queue_size=10)

rospy.sleep(1.0)
r = rospy.Rate(1)
while not rospy.is_shutdown():
    right_ft = (r_forefoot_force + r_sole_force) / 2
    left_ft = (l_forefoot_force + l_sole_force) / 2
    l_foot_Fz_pub.publish(Float64(left_ft))
    r_foot_Fz_pub.publish(Float64(right_ft))
    r.sleep()