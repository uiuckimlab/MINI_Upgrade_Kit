#!/usr/bin/env python3
import rospy

class MINI_CoM: 
    
    def __init__(self):
        rospy.init_node('mini_com', anonymous=True)
    
    
    def computeCoM(self):
        """
        TODO: Calculate whole body CoM 
        """
        pass

    def visualize_CoM(self):
        """
            TODO: Visualize whole body CoM in Rviz
        """
        pass
             
if __name__ == '__main__':
    mini_com = MINI_CoM()