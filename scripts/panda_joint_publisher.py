#!/usr/bin/env python
"""
    Author: Yoojin Oh   
    yoojin.oh@ipvs.uni-stuttgart.de

"""
import numpy as np
import rospy
from sensor_msgs.msg import JointState


class PandaJointPublisher():
    def __init__(self, ConfigWorld, RealWorld):
        """
        Args:
            ConfigWorld ([type]): C
            RealWorld ([type]): RealWorld
        """
        self.pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        
        self.ConfigWorld = ConfigWorld
        self.RealWorld = RealWorld

    def publish(self):
        joints = self.ConfigWorld.getJointNames() \
                +["R_panda_finger_joint1", "R_panda_finger_joint2"] \
                +["L_panda_finger_joint1", "L_panda_finger_joint2"]
                
        configs =  list(self.ConfigWorld.getJointState()) \
                + self._compute_finger_joint_configs("R") \
                + self._compute_finger_joint_configs("L")

        self.msg = JointState()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.name = joints
        self.msg.position = configs
        self.msg.velocity = []
        self.msg.effort = []

        self.pub.publish(self.msg)

    def _compute_finger_joint_configs(self,side):
        
        """
        Args:
            side (string): either "R" or "L"

        Returns:
            list: list of finger joint configrations for one arm
        """
        finger_1 = self.RealWorld.getFrame(side+"_panda_finger_joint1").getPosition()
        finger_2 = self.RealWorld.getFrame(side+"_panda_finger_joint2").getPosition()
        distance =  np.sqrt((finger_1[0]-finger_2[0])**2 \
                        +(finger_1[1]-finger_2[1])**2 \
                        +(finger_1[2]-finger_2[2])**2)
        return [distance/2., distance/2.]