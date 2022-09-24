#!/usr/bin/env python3
import rospy
from tkinter import W
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
from robotis_mini import RobotisMini

from dynamic_reconfigure.server import Server
from robotis_mini_control.cfg import RobotisMiniConfig

y_foot_pos = 0

def callback(config, level):
    global y_foot_pos

    print(config)
    y_foot_pos = config['y_foot_pos']
    execute_static_foot_position(robot, x_offset=0, y_offset=y_foot_pos, z_height=30)
    return config

def execute_static_foot_position(robot, x_offset, y_offset, z_height):
    """
    TODO: Implement static foot position input
    """
    # joint_values_left_hand = robot.ik_left_hand(robot.x_LH0+50, robot.y_LH0-100, robot.z_LH0-90)
    # joint_values_right_hand = robot.ik_right_hand(robot.x_RH0+50, robot.y_RH0+100, robot.z_RH0-90)
    joint_values_right_hand = [0, 0, 0]
    joint_values_left_hand = [0, 0, 0]
    # y +/- 20, x +/- 20
    joint_values_right_foot = robot.ik_right_foot(robot.x_RF0+x_offset, robot.y_RF0+y_offset, robot.z_RF0+z_height, robot.roll_RF0, robot.pitch_RF0)
    joint_values_left_foot = robot.ik_left_foot(robot.x_LF0+x_offset, robot.y_LF0+y_offset, robot.z_LF0+z_height, robot.roll_LF0, robot.pitch_LF0)

    joint_pos_values = joint_values_right_hand + joint_values_left_hand \
                        + joint_values_right_foot + joint_values_left_foot
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = ['r_shoulder_joint','r_biceps_joint', 'r_elbow_joint', 
                            'l_shoulder_joint','l_biceps_joint', 'l_elbow_joint', 
                            'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
                            'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint']

    point = JointTrajectoryPoint()                  
    point.positions = joint_pos_values
    point.time_from_start = rospy.Duration(1.0)
    traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)
    rospy.sleep(1.0)

def execute_variable_foot_position(robot):
    """
    TODO: Implement variable foot position input
    """
    pass

if __name__ == '__main__':
    rospy.init_node('assignment2')
    robot = RobotisMini()
    is_static = True

    srv = Server(RobotisMiniConfig, callback)

    # if is_static:
    #     execute_static_foot_position(robot, x_offset=0, y_offset=-0, z_height=30)
    # else:
    #     execute_variable_foot_position(robot)
    
    rospy.spin()