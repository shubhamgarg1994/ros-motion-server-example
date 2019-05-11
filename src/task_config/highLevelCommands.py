#!/usr/bin/env python

import rospy

from task_states import TaskRunning, TaskDone
from task_commands import *

from motions_server.abstract_task import AbstractTask

class uavTakeOff(AbstractTask):
	def __init__(self, request):
		rospy.loginfo('Take off task being constructed')
		self.status = 0
		self.altitude = request.pose.pose.position.z

	def get_desired_command(self):
		if(self.status == 0):
			self.status = 1
			return TaskRunning(), FlightMode(mode='OFFBOARD',arm=1,altitude=self.altitude)
		else:
			return TaskDone(), StringCommand(data='Task done')


	def cancel(self):
		return True

class uavLand(AbstractTask):
	def __init__(self, request):
		rospy.loginfo('Land task being constructed')
		self.status = 0

	def get_desired_command(self):
		if(self.status == 0):
			self.status = 1
			return TaskRunning(), FlightMode(mode='AUTO.LAND',arm=0,altitude=0)
		else:
			return TaskDone(), StringCommand(data='Task done')


	def cancel(self):
		return True

class uavWaypoint(AbstractTask):
	def __init__(self, request):
		rospy.loginfo('Waypoint task constructed')
		self.status = 0
		self.target_pose = request.pose

	def get_desired_command(self):
		if(self.status == 0):
			self.status = 1
			return TaskRunning(), targetPoint(pose=self.target_pose)
		else:
			return TaskDone(), StringCommand(data='Task done')


	def cancel(self):
		return True

class markerTrack_xy(AbstractTask):
	def __init__(self, request):
		rospy.loginfo('Marker Tracking - xy being constructed')
		self.status = 0
		self.altitude = 1.0

	def get_desired_command(self):
		if(self.status == 0):
			self.status = 1
			return TaskRunning(), TrackMode(altitude=self.altitude, tag='tag_3')
		else:
			return TaskDone(), StringCommand(data='Task done')


	def cancel(self):
		return True