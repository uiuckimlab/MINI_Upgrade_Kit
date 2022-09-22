#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped


odom_pose = []

def model_state_callback(msg):
    global odom_pose 
    odom_pose = msg.pose[1]

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf2_ros.TransformBroadcaster()
model_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, model_state_callback)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

rospy.sleep(1.0)
r = rospy.Rate(100)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = odom_pose.position.x
    transform.transform.translation.y = odom_pose.position.y
    transform.transform.translation.z = odom_pose.position.z
    transform.transform.rotation.x = odom_pose.orientation.x
    transform.transform.rotation.y = odom_pose.orientation.y
    transform.transform.rotation.z = odom_pose.orientation.z
    transform.transform.rotation.w = odom_pose.orientation.w

    odom_broadcaster.sendTransform(transform)

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = odom_pose

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()