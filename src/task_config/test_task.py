#!/usr/bin/env python

import rospy

from task_states import TaskRunning, TaskDone
from task_commands import StringCommand

from motions_server.abstract_task import AbstractTask


class TestTask(AbstractTask):
    def __init__(self, request):
        rospy.loginfo('Task being constructed')
        self._iterations = 0

    def get_desired_command(self):
        if self._iterations > 2000:
            return TaskDone(), StringCommand(data='Task done')
        else:
            self._iterations = self._iterations + 1
            return TaskRunning(), StringCommand(data='TaskRunning')

    def cancel(self):
        return True
