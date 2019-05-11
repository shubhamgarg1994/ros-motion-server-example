#!/usr/bin/env python

import rospy
import actionlib
import time

from robot_motions_server_ros.msg import TaskRequestAction, TaskRequestGoal
from geometry_msgs.msg import PoseStamped

def getWaypoint(x,y,z,yaw=0):
    target_pose = PoseStamped()
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    """
    Please note storing the yaw angle as a 
    orientation along z axis, probably figure out
    a better way to send
    Yaw rate is fixed
    Yaw in radians please
    """
    target_pose.pose.orientation.z = yaw
    return target_pose


def run(server_name):
    client = actionlib.SimpleActionClient(server_name, TaskRequestAction)

    rospy.loginfo('TaskActionClient: Waiting for server')
    client.wait_for_server()
    rospy.loginfo('TaskActionClient: server ready')
    time.sleep(60)
    request = TaskRequestGoal(action_type='uavTakeOff')
    """
    Give me the takeoff altitude
    in the pose
    """
    altitude = 1.5
    request.pose = getWaypoint(0,0,altitude)
    client.send_goal_and_wait(request)

    rospy.logwarn('Success: {}'.format(client.get_result().success))

    #hover for 2 seconds
    time.sleep(2)

    """
    Waypoint following code
    """
    request = TaskRequestGoal(action_type='uavWaypoint')
    request.pose = getWaypoint(0.2,0,altitude,0)

    rospy.loginfo(str(request))
    client.send_goal_and_wait(request)

    rospy.logwarn('Success: {}'.format(client.get_result().success))

    rospy.loginfo('Testing Cancel')

    #hover for 2 seconds
    #time.sleep(2)
    #request = TaskRequestGoal(action_type='uavWaypoint')
    #request.pose = getWaypoint(0.2,0.2,altitude,0)

    #rospy.loginfo(str(request))
    #client.send_goal_and_wait(request)

    #rospy.logwarn('Success: {}'.format(client.get_result().success))

    #rospy.loginfo('Testing Cancel')

    #hover for 2 seconds
    #time.sleep(2)
    #request = TaskRequestGoal(action_type='uavWaypoint')
    #request.pose = getWaypoint(0,0.2,altitude,0)

    #rospy.loginfo(str(request))
    #client.send_goal_and_wait(request)

    #rospy.logwarn('Success: {}'.format(client.get_result().success))

    #rospy.loginfo('Testing Cancel')

    #hover for 2 seconds
    time.sleep(2)
    request = TaskRequestGoal(action_type='uavWaypoint')
    request.pose = getWaypoint(0,0,0.2,0)

    rospy.loginfo(str(request))
    client.send_goal_and_wait(request)

    rospy.logwarn('Success: {}'.format(client.get_result().success))

    rospy.loginfo('Testing Cancel')

    #hover for 2 seconds
    time.sleep(2)

    """
    Landing mode
    """

    #request = TaskRequestGoal(action_type='uavLand')
    #rospy.loginfo(str(request))
    #client.send_goal_and_wait(request)

    #rospy.logwarn('Success: {}'.format(client.get_result().success))

    #rospy.loginfo('Testing Cancel')

if __name__ == '__main__':
    rospy.init_node('test_take-0ff')
    rospy.loginfo('Test Action Client starting up')
    name = rospy.get_param('/task_manager/action_server_name')
run(name)
