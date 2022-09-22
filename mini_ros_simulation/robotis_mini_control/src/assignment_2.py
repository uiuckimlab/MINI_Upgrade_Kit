#!/usr/bin/env python3
import rospy
from robotis_mini import RobotisMini

def execute_static_foot_position(robot):
    """
    TODO: Implement static foot position input
    """
    pass


def execute_variable_foot_position(robot):
    """
    TODO: Implement variable foot position input
    """
    pass

if __name__ == '__main__':
    rospy.init_node('assignment2')
    robot = RobotisMini()
    is_static = False

    if is_static:
        execute_static_foot_position(robot)
    else:
        execute_variable_foot_position(robot)
    
    rospy.spin()