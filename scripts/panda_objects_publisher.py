#!/usr/bin/env python
import numpy as np
import rospy
# from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray


class PandaObjectsPublisher():
    def __init__(self, ConfigWorld, RealWorld):
        """
        Args:
            ConfigWorld ([type]): C
            RealWorld ([type]): RealWorld
        """
        self.obj_pub = rospy.Publisher("objects", MarkerArray, queue_size=10)
        
        self.ConfigWorld = ConfigWorld
        self.RealWorld = RealWorld

    def publish(self):
        arr_marker = MarkerArray()

        # fill list of objects names
        l_objs = 

        for i, o in enumerate(l_objs):
            ob = self.RealWorld.getFrame(o)
            # you can access all the objects properties (color, size, shape, etc.)
            # using the following
            info = ob.info()
            
            # fill in marker properties
            m = Marker()
            m.action = Marker.ADD
            m.header.frame_id = '/base'
            m.header.stamp = rospy.Time.now()
            m.ns = 'marker'
            m.id = i
            
            # fill in marker type and size
            # http://wiki.ros.org/rviz/DisplayTypes/Marker
            # Marker.CYLINDER, Marker.CUBE, Marker.SPHERE etc.
            m.type = marker_type
            m.scale.x =
            m.scale.y = 
            m.scale.z = 

            # object quaternions
            m.pose.orientation.w = 
            m.pose.orientation.x = 
            m.pose.orientation.y = 
            m.pose.orientation.z = 
            
            # marker positions
            m.pose.position.x = 
            m.pose.position.y = 
            m.pose.position.z = 
            
            # marker color
            m.color.r = 
            m.color.g = 
            m.color.b = 
            m.color.a = 

            arr_marker.markers.append(m)
        self.obj_pub.publish(arr_marker)
