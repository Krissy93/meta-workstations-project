#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from actions_library import *
from utils import *

ROBOT_PUB = '/joint_state_pose'
ROBOT_FEEDBACK = '/joint_states'

class Operation():
    def __init__(self):
        self.trajectory = JointTrajectory()
        self.request = commandRequest()
        self.empty = [0,0,0,0,0,0]
        self.pub = rospy.Publisher(ROBOT_PUB, JointTrajectory, queue_size = 1, latch=True)
        self.goal = False
        self.last_position = None

    def feedbackCallback(self, msg):
        """ Callback function that reads the feedback written by the robot in the feedback topic.
        Be careful since the feedback is written continuosly, so the trick with the stamp to align
        the messages do not work. In this case it is better to check here if the feedback received
        corresponds to the theoretical position the system is waiting for.

        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort"""

        # to know if the robot reached the goal, the callback must check
        # if the position obtained from the feedback (msg.position) has a value
        # between the theoretical position +- 0.01 radiants. This is a safety measure
        # because it is nearly impossible to reach the theoretical position requested
        # so an approximation is needed
        if (msg.position >= self.trajectory.positions - 0.01) and (msg.position <= self.trajectory.positions + 0.01):
            # if the two positions are almost equal, it means that the robot
            # has reached the goal of the last message. Sets the goal to true
            self.goal = True
        else:
            self.goal = False
        # regardless of the goal, it updates the position according to what is read on the topic
        self.last_position = msg.position

    def execute_operation(self):
        ''' This portion will be automatically filled according to the executed set of actions. '''

        init_trajectory(self.trajectory, self.empty)
