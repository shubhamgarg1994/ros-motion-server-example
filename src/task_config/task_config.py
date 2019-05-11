#!/usr/bin/env python

"""
System level imports
"""
import rospy
import tf
import numpy as np
from numpy.linalg import inv
import quaternion
import sys, signal
import time
import math

"""
Import messages
"""
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import *
from mavros_msgs.srv import *

"""
Import Action server stuff
"""
from highLevelCommands import *
from task_states import TaskRunning, TaskDone
from motions_server.abstract_command_handler import (TaskHandledState,
                                                     AbstractCommandHandler)
from motions_server.abstract_safety_responder import AbstractSafetyResponder

class CommandHandler(AbstractCommandHandler):
    def __init__(self):

        #initailizing the services and publishers/subscribers
        rospy.wait_for_service('mavros/cmd/arming')
        rospy.wait_for_service('mavros/cmd/takeoff')
        rospy.wait_for_service('mavros/set_mode')

        try:
            self._armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            self._flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)

        self._sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self._local_pos = Odometry()
        self._taskname = None
        self._rate = rospy.Rate(20.0)
        self._state = State()
        self._vision_pose = PoseStamped()
        self._camera_pose = None
        self.tfmessage = None

        self._vision_pose_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1000)

        rospy.Subscriber('mavros/state', State, self.stateCb)
        rospy.Subscriber('mavros/local_position/odom', Odometry, self.odomCb)
        rospy.Subscriber("/tf", TFMessage, self.tfCb)

        ##---------COmpute the HT from imu-camera---------------#
        translation_matrix = [-0.05, 0, 0.24638]
        translation_matrix = np.array(translation_matrix)
        translation_matrix = translation_matrix.reshape(1,3)
        rotation_angles = [0.5 * np.pi/180, 5 * np.pi / 180.0, 90 * np.pi/180]
        alpha = rotation_angles[2]
        beta = rotation_angles[1]
        gamma = rotation_angles[0]
        last_row = np.array([0.0,0.0,0.0,1.0]).reshape(1,4)
        rotation_matrix = np.array([[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma)],
                                    [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)],
                                    [-np.sin(beta)             , np.cos(beta)*np.sin(gamma)                                            , np.cos(beta)*np.cos(gamma)]])
        
        self.ht = np.concatenate((rotation_matrix,translation_matrix.transpose()),axis=1)
        self.ht = np.concatenate((self.ht,last_row),axis=0)

    def stateCb(self, msg):
        self._state = msg

    def odomCb(self, msg):
        self._local_pos = msg

    def tfCb(self, msg):
        tr = tf.TransformerROS()
        self.tfmessage = msg.transforms[0]
        tr.setTransform(self.tfmessage)
        self._camera_pose = tr
        
    def check_request(self, request):
        if request.action_type == 'uavTakeOff':
            self._taskname = request.action_type
            return uavTakeOff(request)
        elif request.action_type == 'uavLand':
            self._taskname = request.action_type
            return uavLand(request)
        elif request.action_type == 'markerTrack_xy':
            self._taskname = request.action_type
            return markerTrack_xy(request)
        elif request.action_type == 'uavWaypoint':
            self._taskname = request.action_type
            return uavWaypoint(request)
        else:
            self._taskname = None
            return None

    def wait_until_ready(self, timeout):
        return

    def handle(self, command, state):
        if isinstance(state, TaskRunning):
            self._handle_task_running(command)
            return TaskHandledState.OKAY
        elif isinstance(state, TaskDone):
            self._handle_task_done(command)
            return TaskHandledState.DONE
        else:
            rospy.logerr('CommandHandler: Task provided invalid state')
            return TaskHandledState.ABORTED

    def _handle_task_running(self, command):
        rospy.loginfo("Handle task running: ")
        if(self._taskname == "uavTakeOff"):
            if(command.arm>0 and command.altitude>0):
                
                #Arming the vehicle
                try:
                    self._armService(bool(command.arm))
                except rospy.ServiceException, e:
                    rospy.logerr("Service arming call failed: %s"%e)
                
                #Put the vehicle in offboard mode
                # We need to send few setpoint messages, then activate OFFBOARD mode, to take effect

                k=0
                while k<10:
                    self._sp_pub.publish(command.sp)
                    self._rate.sleep()
                    k = k + 1

                try:
                    self._flightModeService(custom_mode=command.mode)
                except rospy.ServiceException, e:
                    rospy.logger("service set_mode call failed: %s. Offboard Mode could not be set."%e)

                command.sp.position.z = command.altitude

                while(abs(self._local_pos.pose.pose.position.z - command.altitude) > 0.2):
                    self._sp_pub.publish(command.sp)
                    self._rate.sleep()

        elif(self._taskname == "uavLand"):
            
            try:
                self._flightModeService(custom_mode=command.mode)
            except rospy.ServiceException, e:
                rospy.logger("service set_mode call failed: %s. Offboard Mode could not be set."%e)
           
            rospy.loginfo("UAV Land and Disarm successful")
        
        elif(self._taskname == "markerTrack_xy"):
            mat = np.zeros((4,4))
            #print msg
            k = 0
            while k<20:
                try:
                    mat = self._camera_pose.asMatrix(command.tag, self.tfmessage.header)
                    output_matrix = np.matmul(mat,inv(self.ht))
                    quaternion_msg = quaternion.as_float_array(quaternion.from_rotation_matrix(output_matrix[0:4,0:4]))
                    self._vision_pose.header.stamp = rospy.get_rostime()
                    self._vision_pose.pose.position.x = -1*output_matrix[0,3]
                    self._vision_pose.pose.position.y = -1*output_matrix[1,3]
                    self._vision_pose.pose.position.z = command.altitude
                    self._vision_pose.pose.orientation.x = quaternion_msg[0]
                    self._vision_pose.pose.orientation.y = quaternion_msg[1]
                    self._vision_pose.pose.orientation.z = quaternion_msg[2]
                    self._vision_pose.pose.orientation.w = quaternion_msg[3]

                    self._vision_pose_pub.Publish(self._vision_pose)
                
                except Exception as e:
                    print e
                    pass
                self._rate.sleep()
                k += 1

            rospy.loginfo("Marker Tracking successful")
        
        elif(self._taskname == "uavWaypoint"):

            x1 = self._local_pos.pose.pose.position.x
            y1 = self._local_pos.pose.pose.position.y
            z1 = self._local_pos.pose.pose.position.z

            x2 = command.sp.position.x
            y2 = command.sp.position.y
            z2 = command.sp.position.z

            while(euclidianDistance(x1, y1, z1, x2, y2, z2) > 0.2):
                self._sp_pub.publish(command.sp)
                self._rate.sleep()
                x1 = self._local_pos.pose.pose.position.x
                y1 = self._local_pos.pose.pose.position.y
                z1 = self._local_pos.pose.pose.position.z

                
            rospy.loginfo("UAV Waypoint navigation task done")

    def _handle_task_done(self, command):
        rospy.loginfo("Handle task done called: "+str(command))        

def euclidianDistance(x, y, z, u, v, w):
    return math.sqrt(pow((x-u),2) + pow((y-v),2) + pow((z-w),2))

class SafetyResponder(AbstractSafetyResponder):
    def __init__(self):
        pass

    def activate_safety_response(self):
        return

    def wait_until_ready(self, timeout):
        return
