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

start_time = 0 # sec
end_time = 30 # sec
control_period_ = 0.001 # ms
freq_n = 0.25 # hz
amplitude = 56 # mm

signal_type_ = 'triangle' # sine or triangle

def callback(config, level):
    global x_foot_pos
    global y_foot_pos
    global z_foot_pos

    x_foot_pos = config['x_foot_pos']
    y_foot_pos = config['y_foot_pos']
    z_foot_pos = config['z_foot_pos']

    execute_static_foot_position(robot, x_foot_pos=x_foot_pos, 
                                        y_foot_pos=y_foot_pos, 
                                        z_foot_pos=z_foot_pos, 
                                        time = 1.0)

    print("x_foot_pos: ", x_foot_pos, 
        "\ny_foot_pos: ", y_foot_pos,
        "\nz_foot_pos: ", z_foot_pos,
        "\n--------------------")

    return config

def sine_wave_input():
    global start_time
    global end_time 
    global control_period_
    global freq_n
    global amplitude
    theta = 0
 
    time = np.arange(start_time, end_time, control_period_)
    sinewave = amplitude * np.sin(2 * np.pi * freq_n * time + theta)

    plt.plot(time, sinewave)
    plt.show()

    return sinewave

def triangle_wave_input(): 
    global start_time
    global end_time 
    global control_period_
    global freq_n
    global amplitude
    theta = 0

    time = np.arange(start_time, end_time, control_period_)
    sawtooth_wave = amplitude * signal.sawtooth(2 * np.pi * freq_n * time + theta, width=0.5)

    plt.plot(time, sawtooth_wave)
    plt.show()

    return sawtooth_wave

def execute_static_foot_position(robot, x_foot_pos, y_foot_pos, z_foot_pos, time):
    """
    TODO: Implement static foot position input
    """
    joint_values_right_hand = [0, 0, 0]
    joint_values_left_hand = [0, 0, 0]
    joint_values_right_foot = robot.ik_right_foot(robot.x_RF0+x_foot_pos, robot.y_RF0+y_foot_pos, z_foot_pos, robot.roll_RF0, robot.pitch_RF0)
    joint_values_left_foot = robot.ik_left_foot(robot.x_LF0+x_foot_pos, robot.y_LF0+y_foot_pos, z_foot_pos, robot.roll_LF0, robot.pitch_LF0)

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

def execute_variable_foot_position(robot, z_foot_pos):
    """
    TODO: Implement variable foot position input
    """
    global control_period_
    global signal_type_

    if signal_type_ == 'sine':
        wave_signal = sine_wave_input()
    else:
        wave_signal = triangle_wave_input()

    time_from_start = 0.0
    x_offset = 0

    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = ['r_shoulder_joint','r_biceps_joint', 'r_elbow_joint', 
                            'l_shoulder_joint','l_biceps_joint', 'l_elbow_joint', 
                            'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
                            'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint']

    for i in range(len(wave_signal)):   
        y_offset = wave_signal[i]
        joint_values_right_hand = [0, 0, 0]
        joint_values_left_hand = [0, 0, 0]
        joint_values_right_foot = robot.ik_right_foot(robot.x_RF0+x_offset, robot.y_RF0+y_offset, z_foot_pos, robot.roll_RF0, robot.pitch_RF0)
        joint_values_left_foot = robot.ik_left_foot(robot.x_LF0+x_offset, robot.y_LF0+y_offset, z_foot_pos, robot.roll_LF0, robot.pitch_LF0)

        joint_pos_values = joint_values_right_hand + joint_values_left_hand \
                            + joint_values_right_foot + joint_values_left_foot
        

        point = JointTrajectoryPoint()                  
        point.positions = joint_pos_values
        
        if i == 0:
            time_from_start = 5.0
        else:
            time_from_start += control_period_ 

        point.time_from_start = rospy.Duration(time_from_start)
        traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)
        
if __name__ == '__main__':
    rospy.init_node('assignment2')

    robot = RobotisMini()
    # init dynamic reconfigure server 
    srv = Server(RobotisMiniConfig, callback)

    robot.init_pose(z_foot_pos = -166)

    # execute_variable_foot_position(robot, z_foot_pos= -166)
    
    rospy.spin()