#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs as tf_geo
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

class MINI_CoM: 
    
    def __init__(self):
        rospy.init_node('mini_com', anonymous=True)

        self.base_link_frame = rospy.get_param("~base_link_frame", "base_link")
        self.total_mass = 0
        #get robot description from URDF
        robot = URDF.from_parameter_server()
        self.links = robot.link_map
        self.inertial_dict = {}

        #Delete links, which contain no mass description
        unnecessary_links = []
        for link in self.links:
            if self.links[link].inertial == None or self.links[link].inertial.origin == None:
                unnecessary_links.append(link)

        for link in unnecessary_links:
            del self.links[link]
        
        #Calculate the total mass of the robot
        for link in self.links:
            self.total_mass += self.links[link].inertial.mass
            self.inertial_dict[link] = self.links[link].inertial.to_yaml()

        rospy.loginfo("Mass of robot is %f", self.total_mass)

        self.init_marker()

    def init_marker(self):
        marker = Marker()
        marker.header.frame_id = self.base_link_frame
        marker.header.stamp = rospy.Time()
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        self.marker = marker
    
    def start(self):
        """
        TODO: Calculate whole body CoM 
        """
        #initializations for tf and marker
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        P_CoM_link = geometry_msgs.msg.PointStamped()
        
        pub = rospy.Publisher('com', Marker, queue_size=1)
        
        rate = rospy.Rate(100)
        rospy.sleep(2.0)
        
        #loop for calculating the CoM while robot is not shutdown
        while not rospy.is_shutdown():
            x = 0
            y = 0
            z = 0
            for link in self.links:
                try:
                    #get transformation matrix of link (target, source)
                    trans = tfBuffer.lookup_transform(self.base_link_frame, link, rospy.Time())

                    #transform CoM of link
                    #   m_i = self.links[link].inertial.mass
                    #   P_CoM_n = self.links[link].inertial.origin.xyz -> tf link to base frame
                    P_CoM_link.point.x = self.links[link].inertial.origin.xyz[0]
                    P_CoM_link.point.y = self.links[link].inertial.origin.xyz[1]
                    P_CoM_link.point.z = self.links[link].inertial.origin.xyz[2]
                    P_CoM_link.header.frame_id = link
                    P_CoM_link.header.stamp = rospy.get_rostime()

                    P_CoM_n = tf_geo.do_transform_point(P_CoM_link, trans)
                    m_i = self.links[link].inertial.mass
                    
                    #calculate part of CoM equation depending on link
                    x += m_i * P_CoM_n.point.x
                    y += m_i * P_CoM_n.point.y
                    z += m_i * P_CoM_n.point.z

                except tf2_ros.TransformException as err:
                    rospy.logerr("TF error in COM computation %s", err)

                
            #finish CoM calculation
            x = x/self.total_mass
            y = y/self.total_mass
            z = z/self.total_mass

            #send CoM position to RViZ
            self.marker.pose.position.x = x
            self.marker.pose.position.y = y
            self.marker.pose.position.z = z
            pub.publish(self.marker)

            try:
                rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                rospy.logwarn("Moved backwards in time.")

    def visualize_CoM(self):
        """
            TODO: Visualize whole body CoM in Rviz
        """
        pass
             
if __name__ == '__main__':
    mini_com = MINI_CoM()
    mini_com.start()