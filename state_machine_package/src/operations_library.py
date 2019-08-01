#!/usr/bin/env python

import rospy
from robot import *
from utils import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

PKG_PATH = '/home/eulero/projects/ros_ws/src/state_machine_package/'

def op_list():
    """ Prints all the available operations present in this file. """

    rospy.loginfo(color.BOLD + color.PURPLE + '|----------------------|' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| AVAILABLE OPERATIONS |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 1: MOKA EVAL TEST    |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '|----------------------|' + color.END)

    operations = ['MOKA']

    return operations
    
def check_feed(self, gripper):
    goal = False
    while (goal == False):
        if gripper == 0:
            # until goal not reached, checks the feedback topic
            self.trajsub = rospy.Subscriber('/ur10/joint_states', JointState, self.feedbackCallback, queue_size = 1)
        else:
            # if the action is a gripper one, sets the goal to true automatically
            goal = True
     return goal

def moka_evaluation_op(pub, trajectory, empty):
    """ Loads the XYZ points file and calculates the joints coordinates according
    to the robot-dependant inverse kinematic function. This function is defined in robot.py """

    # loads the Qpoints file
    file = load_txt(PKG_PATH + 'data/QPoints.txt')
    names, points = load_points(file)
    rospy.loginfo(color.BOLD + color.YELLOW + '-- POINTS LOADED -- + color.END)
   
    #### MOVES TO BASE POSITION ####
    move_to_point(pub, trajectory, points[0], empty, empty, empty)

    #### OPENS THE GRIPPER ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # opens gripper now
        control_gripper(1)
    else:
        return ERROR
        
    #### MOVES TO FIRST POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to second point (reaches the object)
        move_to_point(pub, trajectory, points[1], empty, empty, empty)
    else:
        return ERROR
        
    #### GOES DOWN A BIT TO PICK THE OBJECT CORRECTLY ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to second point (reaches the object)
        move_to_point(pub, trajectory, points[2], empty, empty, empty)
    else:
        return ERROR
        
    #### CLOSES THE GRIPPER TO PICK UP THE OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # closes the gripper and takes up the object
        control_gripper(0)
    else:
        return ERROR
        
    #### MOVES TO "RELEASE" POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to third point (reaches the pick down object point)
        move_to_point(pub, trajectory, points[3], empty, empty, empty)
    else:
        return ERROR
        
    #### GOES DOWN A BIT TO RELEASE OBJECT SMOOTHLY ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        move_to_point(pub, trajectory, points[4], empty, empty, empty) ##########
    else:
        return ERROR
        
    #### OPENS GRIPPER TO RELEASE OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # opens gripper now to release object
        control_gripper(1)
    else:
        return ERROR
        
    #### GOES UP A BIT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        move_to_point(pub, trajectory, points[4], empty, empty, empty) ##########
    else:
        return ERROR
        
    #### MOVES TO SECOND OBJECT POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to fourth point
        move_to_point(pub, trajectory, points[4], empty, empty, empty)
    else:
        return ERROR
        
    #### MOVES DOWN TO PICK UP OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to fifth point
        move_to_point(pub, trajectory, points[5], empty, empty, empty)
    else:
        return ERROR
        
    #### CLOSES GRIPPER TO PICK UP OBJECT (OPENED ALREADY) ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # closes gripper now
        control_gripper(0)
    else:
        return ERROR
        
    #### MOVES TO "RELEASE" POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to third point (reaches the pick down object point)
        move_to_point(pub, trajectory, points[3], empty, empty, empty)
    else:
        return ERROR
        
    #### MOVES DOWN A BIT TO RELEASE OBJECT SMOOTHLY ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to sixth point
        move_to_point(pub, trajectory, points[6], empty, empty, empty)
    else:
        return ERROR
        
    #### OPENS GRIPPER TO RELEASE OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # opens gripper now
        control_gripper(1)
    else:
        return ERROR
        
    #### GOES UP A BIT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        move_to_point(pub, trajectory, points[4], empty, empty, empty) ##########
    else:
        return ERROR
        
    #### MOVES TO THIRD OBJECT POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to seventh point
        move_to_point(pub, trajectory, points[7], empty, empty, empty)
    else:
        return ERROR
        
    #### MOVES DOWN TO PICK UP OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to seventh point
        move_to_point(pub, trajectory, points[7], empty, empty, empty)
    else:
        return ERROR
        
    #### CLOSES GRIPPER TO PICK UP OBJECT (OPENED ALREADY) ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # closes gripper now
        control_gripper(0)
    else:
        return ERROR
        
    #### MOVES TO "RELEASE" POSITION ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to third point (reaches the pick down object point)
        move_to_point(pub, trajectory, points[3], empty, empty, empty)
    else:
        return ERROR
        
    #### MOVES DOWN A BIT TO RELEASE OBJECT SMOOTHLY ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # moves to sixth point
        move_to_point(pub, trajectory, points[6], empty, empty, empty)
    else:
        return ERROR
        
    #### OPENS GRIPPER TO RELEASE OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        # opens gripper now
        control_gripper(1)
    else:
        return ERROR
        
    #### GOES UP TO LET OPERATOR ASSEMBLE OBJECT ####
    goal = False
    goal = check_feed(self, 0)
    if goal == True:
        move_to_point(pub, trajectory, points[4], empty, empty, empty) ##########
    else:
        return ERROR
