#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

from scipy import signal
import matplotlib.pyplot as plt
import numpy as np
import math

class RobotisMini:
    def __init__(self):
        self.x_RH0, self.y_RH0, self.z_RH0, self.x_LH0, self.y_LH0, self.z_LH0 = 0.0, -174.00, -12.00, 0.0, 174.0, -12.0
        self.x_RF0, self.y_RF0, self.z_RF0, self.roll_RF0, self.pitch_RF0 = 15.0, -33.0, -196.0, 0.0, 0.0
        self.x_LF0, self.y_LF0, self.z_LF0, self.roll_LF0, self.pitch_LF0 = 15.0, 33.0, -196.0, 0.0, 0.0

        self.execute_pub = rospy.Publisher("/robotis_mini/full_body_controller/command", JointTrajectory, queue_size=100)
        self.jointTrajectoryControllerStateSubscriber = rospy.Subscriber("/robotis_mini/full_body_controller/state", JointTrajectoryControllerState, self.callback_state)
        rospy.sleep(1.0)

        # Initalize subscriber to robotis_mini/joint_states and callback function 
        # each time data is published
        sub_joint_state = rospy.Subscriber('robotis_mini/joint_states', JointState, self.joint_state_callback)


    def callback_state(self, msg):
        self.robot_state = msg

    def joint_state_callback(self, msg):

        self.current_position = msg.position
        self.current_velocity = msg.velocity
        self.current_effort = msg.effort

        print("l_ankle_joint position: ", self.current_position[0],
                "\nl_ankle_joint velocity:", self.current_velocity[0],
                "\nl_ankle_joint effort", self.current_effort[0],
                "\n-------------------")

    def init_pose(self, z_height):
        z_height = 30
        joint_values_right_hand = [0, 0, 0]
        joint_values_left_hand = [0, 0, 0]
        joint_values_right_foot = self.ik_right_foot(self.x_RF0, self.y_RF0, self.z_RF0+z_height, self.roll_RF0, self.pitch_RF0)
        joint_values_left_foot = self.ik_left_foot(self.x_LF0, self.y_LF0, self.z_LF0+z_height, self.roll_LF0, self.pitch_LF0)

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
        point.time_from_start = rospy.Duration(3.0)
        traj_msg.points.append(point)

        self.execute_pub.publish(traj_msg)
        rospy.sleep(3.0)

    def ik_right_hand(self, x_RH, y_RH, z_RH):
        L_sh = 39.0 # Origin to arm roll joint
        L_a1 = 18.0 # Shoulder bracket horizontal distance
        L_a2 = 12.0 # Shoulder bracket vertical distance
        L_a3 = 45.0 # Upper arm length
        L_a4 = 72.0 # Lower arm length

        # Right Arm
        x_RH0 = x_RH
        y_RH0 = y_RH + (L_sh + L_a1)
        z_RH0 = z_RH

        th1 = -np.arctan(x_RH0 / (z_RH0 + 0.00001))

        R1_RH = math.sqrt((x_RH0 - L_a2 * np.sin(th1)) * (x_RH0 - L_a2 * np.sin(th1)) \
                    + y_RH0 * y_RH0 + (z_RH0 + L_a2 * np.cos(th1)) * (z_RH0 + L_a2 * np.cos(th1)))

        if R1_RH > 117:
            R1_RH = 117
        else:
            R1_RH = R1_RH

        alpha_RH = np.arccos((L_a3 * L_a3 + L_a4 * L_a4 - R1_RH * R1_RH) / (2 * L_a3 * L_a4))

        # Elbow Joint angle
        th5 = -math.pi + alpha_RH
        R2_RH = math.sqrt((x_RH0 - L_a2 * np.sin(th1)) * (x_RH0 - L_a2 * np.sin(th1)) \
                            + (z_RH0 + L_a2 * np.cos(th1)) * (z_RH0 + L_a2 * np.cos(th1)))

        # Shoulder Joint angle
        if z_RH > 0:
            th3 = math.pi / 2 + (np.arctan(y_RH0 / R2_RH) \
                + np.arccos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)))
        else:
            th3 = -math.pi / 2 + (-np.arctan(y_RH0 / R2_RH) \
                + np.arccos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)))
        
        ret = [th1, th3, th5]
        return ret

    def ik_whole_body_foot_pos(self):
        th_RF = self.ik_right_foot(0, 0, 0, 0, 0)
        th_LF = self.ik_left_foot(0, 0, 0, 0, 0)
        # print(th_RF + th_LF)

    def ik_left_hand(self, x_LH, y_LH, z_LH):
        L_sh = 39.0 # Origin to arm roll joint
        L_a1 = 18.0 # Shoulder bracket horizontal distance
        L_a2 = 12.0 # Shoulder bracket vertical distance
        L_a3 = 45.0 # Upper arm length
        L_a4 = 72.0 # Lower arm length

        # Position of Left hand in Initial Pose (all the left arm motor angle == 0)
        # Left Arm
        x_LH0 = x_LH
        y_LH0 = y_LH - (L_sh + L_a1)
        z_LH0 = z_LH

        th2 = np.arctan(x_LH0 / (z_LH0 + 0.00001))

        # Range of XL320 motor -150 to 150 deg

        R1_LH = math.sqrt((x_LH0 - L_a2 * np.sin(th2)) * (x_LH0 - L_a2 * np.sin(th2)) \
                            + y_LH0 * y_LH0 + (z_LH0 + L_a2 * np.cos(th2)) * (z_LH0 + L_a2 * np.cos(th2)))

        if R1_LH > 117:
            R1_LH = 117
        else:
            R1_LH = R1_LH

        alpha_LH = np.arccos((L_a3 * L_a3 + L_a4 * L_a4 - R1_LH * R1_LH) / (2 * L_a3 * L_a4))

        # Elbow Joint angle
        th6 = math.pi - alpha_LH

        R2_LH = math.sqrt((x_LH0 - L_a2 * np.sin(th2)) * (x_LH0 - L_a2 * np.sin(th2)) + (z_LH0 + L_a2 * np.cos(th2)) * (z_LH0 + L_a2 * np.cos(th2)))

        # Shoulder Joint angle
        if z_LH0 > 0:
            th4 = -((np.arctan(R2_LH / y_LH0) + np.arccos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))))
        else:
            th4 = -(-math.pi / 2 + (np.arctan(y_LH0 / R2_LH) + np.arccos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))))

        ret = [th2, th4, th6]
        # print("ik left hand", ret)
        return ret

    def ik_right_foot(self, x_RF, y_RF, z_RF, th_roll, th_pitch):
        L_by = 24.0  # Origin to pelvis vertical length
        L_bz = 72.0  # Pelvis horizontal length
        L_bx = 15.0  # Shoulder joint axis to Leg center (On Sagittal Plane)
        L_l1 = 6.0   # Pelvis Roll axis to pitch axis
        L_l2 = 45.0  # Thigh Length
        L_l3 = 42.0  # Shank Length
        L_l4 = 31.0  # Ankle Length
        L_f = 9.0    # Foot horizontal length

        pos_RF = [-L_bz - z_RF - L_f * np.sin(th_roll) - L_l4 * np.cos(th_pitch) * np.cos(th_roll),
                    L_by + y_RF + L_f * np.cos(th_roll) - L_l4 * np.cos(th_pitch) * np.sin(th_roll),
                    x_RF - L_bx + L_l4 * np.sin(th_pitch)]

        th7 = np.arctan(pos_RF[1] / pos_RF[0])

        R1_RF = math.sqrt((pos_RF[0] - L_l1 * np.cos(th7)) * (pos_RF[0] - L_l1 \
                            * np.cos(th7)) + (pos_RF[1] - L_l1 * np.sin(th7)) \
                            * (pos_RF[1] - L_l1 * np.sin(th7)) + pos_RF[2] * pos_RF[2])

        alpha_RF = np.arccos((L_l2 * L_l2 + L_l3 * L_l3 - R1_RF * R1_RF) / (2 * L_l2 * L_l3))
        th11 = math.pi - alpha_RF

        R2_RF = math.sqrt((pos_RF[0] - L_l1 * np.cos(th7)) * (pos_RF[0] - L_l1 * np.cos(th7)) \
                            + (pos_RF[1] - L_l1 * np.sin(th7)) * (pos_RF[1] - L_l1 * np.sin(th7)))

        th9 = -((np.arctan(pos_RF[2] / R2_RF) + np.arccos((L_l2 * L_l2 + R1_RF * R1_RF - L_l3 * L_l3) / (2 * L_l2 * R1_RF))))

        th13 = -np.arcsin(np.cos(th9 + th11) * np.cos(th_roll) * np.cos(th7) \
                        * np.sin(th_pitch) - np.sin(th9 + th11) \
                        * np.cos(th_pitch) + np.cos(th9 + th11) \
                        * np.sin(th_pitch) * np.sin(th_roll) * np.sin(th7))

        th15 = -np.arcsin(np.sin(th_roll - th7) * np.cos(th_pitch))

        ret = [th7, th9, th11, -th13, th15]
        return ret

    def ik_left_foot(self, x_LF, y_LF, z_LF, th_roll, th_pitch):
        L_by = 24.0 # Origin to pelvis vertical length
        L_bz = 72.0 # Pelvis horizontal length
        L_bx = 15.0 # Shoulder joint axis to Leg center (On Sagittal Plane)
        L_l1 = 6.0  # Pelvis Roll axis to pitch axis
        L_l2 = 45.0 # Thigh Length
        L_l3 = 42.0 # Shank Length
        L_l4 = 31.0 # Ankle Length
        L_f = 9.0   # Foot horizontal length

        pos_LF = [L_f * np.sin(th_roll) - z_LF - L_bz - L_l4 * np.cos(th_pitch) * np.cos(th_roll),
                    y_LF - L_by - L_f * np.cos(th_roll) - L_l4 * np.cos(th_pitch) * np.sin(th_roll),
                    x_LF - L_bx + L_l4 * np.sin(th_pitch)]

        th8 = np.arctan(pos_LF[1] / pos_LF[0])
        R1_LF = math.sqrt((pos_LF[0] - L_l1 * np.cos(th8)) \
                            * (pos_LF[0] - L_l1 * np.cos(th8)) + (pos_LF[1] - L_l1 * np.sin(th8)) \
                            * (pos_LF[1] - L_l1 * np.sin(th8)) + pos_LF[2] * pos_LF[2])
        alpha_LF = np.arccos((L_l2 * L_l2 + L_l3 * L_l3 - R1_LF * R1_LF) / (2 * L_l2 * L_l3))

        th12 = -math.pi + alpha_LF
        R2_LF = math.sqrt((pos_LF[0] - L_l1 * np.cos(th8)) * (pos_LF[0] - L_l1 * np.cos(th8)) \
                            + (pos_LF[1] - L_l1 * np.sin(th8)) * (pos_LF[1] - L_l1 * np.sin(th8)))

        th10 = (np.arctan(pos_LF[2] / R2_LF) + np.arccos((L_l2 * L_l2 + R1_LF * R1_LF - L_l3 * L_l3) / (2 * L_l2 * R1_LF)))

        th14 = -np.arccos(np.cos(th10 + th12) * np.cos(th_pitch) - np.sin(th10 + th12) \
                            * np.cos(th_roll) * np.cos(th8) * np.sin(th_pitch) - np.sin(th10 + th12) \
                            * np.sin(th_pitch) * np.sin(th_roll) * np.sin(th8))
        th16 = -np.arcsin(np.sin(th_roll - th8) * np.cos(th_pitch))

        ret = [-th8, -th10, -th12, th14, -th16]
        return ret

if __name__ == '__main__':
    rospy.init_node('robotis_mini')
    robot = RobotisMini()
    
    rospy.spin()