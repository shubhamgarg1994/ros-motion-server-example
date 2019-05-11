#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import sys, signal
import time

#------------------
# Flight Mode is the command to change the flight mode
# mode variable stores the flight mode to be set
# arm stores -1 to disarm 0 to not change the arming status and 1 to change to arm
# altitude stores the height
#------------------


class StringCommand(object):
    def __init__(self, data=''):
        self.data = data

class FlightMode(object):
    def __init__(self, mode='', arm=0, altitude=0):
        self.mode = mode
        self.arm = arm
        self.altitude = altitude
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

class Arm(object):
    def __init__(self, arm):
    	if(mode!=""):
        	self.arm = arm
        else:
        	rospy.loginfo("Arm Mode not sent")

class TrackMode(object):
    def __init__(self, altitude, tag="tag_3"):
        self.vision_pose = PoseStamped()
        self.vision_pose.pose.position.z = altitude

class targetPoint(object):
    def __init__(self, pose):
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        self.sp.position = pose.pose.position
        self.sp.yaw = pose.pose.orientation.z
        self.sp.yaw_rate = 0.0