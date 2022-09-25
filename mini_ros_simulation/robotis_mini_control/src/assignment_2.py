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

x_foot_pos = 0
y_foot_pos = 0
z_foot_pos = 0

def callback(config, level):
    global x_foot_pos
    global y_foot_pos
    global z_foot_pos

    x_foot_pos = config['x_foot_pos']
    y_foot_pos = config['y_foot_pos']
    z_foot_pos = config['z_foot_pos']

    execute_static_foot_position(robot, x_offset=x_foot_pos, 
                                        y_offset=y_foot_pos, 
                                        z_height=z_foot_pos, time = 1.0)

    print("x_foot_pos: ", x_foot_pos, 
        "\ny_foot_pos: ", y_foot_pos,
        "\nz_foot_pos: ", z_foot_pos,
        "\n--------------------")

    return config

def sine_wave_input():
    start_time = 0
    end_time = 10
    sample_rate = 1000
    frequency = 0.25
    amplitude = 30
    theta = 0
    time = np.arange(start_time, end_time, 1/sample_rate)
    sinewave = amplitude * np.sin(2 * np.pi * frequency * time + theta)
    plt.plot(time, sinewave)
    return sinewave

def triangle_wave_input(): 
    start_time = 0
    end_time = 10
    sample_rate = 1000
    frequency = 0.25
    amplitude = 30
    theta = 0
    time = np.arange(start_time, end_time, 1/sample_rate)
    sawtooth_wave = amplitude * signal.sawtooth(2 * np.pi * frequency * time + theta, width=0.5)
    plt.plot(time, sawtooth_wave)
    return sawtooth_wave

def execute_static_foot_position(robot, x_offset, y_offset, z_height, time):
    """
    TODO: Implement static foot position input
    """
    joint_values_right_hand = [0, 0, 0]
    joint_values_left_hand = [0, 0, 0]
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
    point.time_from_start = rospy.Duration(time)
    traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)
    rospy.sleep(time)

def execute_variable_foot_position(robot):
    """
    TODO: Implement variable foot position input
    """
    input = triangle_wave_input()
    for i in input:   
        execute_static_foot_position(robot, x_offset=0, 
                                            y_offset=i, 
                                            z_height=35, 
                                            time = 0.01)
        
if __name__ == '__main__':
    rospy.init_node('assignment2')

    robot = RobotisMini()
    # init dynamic reconfigure server 
    srv = Server(RobotisMiniConfig, callback)

    
    execute_variable_foot_position(robot)
    
    rospy.spin()