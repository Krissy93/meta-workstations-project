#!/usr/bin/env python

import rospy
from robot import *
from utils import *

def action_list():
    """ Prints all the available actions present in this file. """

    rospy.loginfo(color.BOLD + color.PURPLE + '| AVAILABLE ACTIONS |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 1: MOVE TO POINT  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 2: OPEN GRIPPER   |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 3: CLOSE GRIPPER  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 4: SCREW          |' + color.END)

    actions = ['MOVE TO POINT', 'OPEN GRIPPER', 'CLOSE GRIPPER', 'SCREW']

    return actions

def init_points():
    """ Loads the XYZ points file and calculates the joints coordinates according
    to the robot-dependant inverse kinematic function. This function is defined in robot.py """

    # loads the points file
    file = load_txt('/home/optolab/smach_ws/src/state_machine_package/data/SPoints.txt')
    names, points = load_points(file)
    # checks for duplicates. Just a safety measure
    names, points = checkifduplicates(names, points)
    # performs the points elaboration in order to obtain the QQ joints coordinates of the XYZ positions.
    # the function calls the corresponding inverse kinematics function of the robot
    Q = elaborateKinMulti(points)
    # writes them in the corresponding file. It is not an append but a complete rewriting of the contents
    QQ = zip(names, Q)
    QPoints = write_txt('/home/optolab/smach_ws/src/state_machine_package/data/QPoints.txt', QQ)
    # debug
    rospy.loginfo(color.BOLD + color.PURPLE + '-- JOINTS COORDINATES RECALCULATED!--' + color.END)
    for i in range(0,len(points)):
        rospy.loginfo(color.BOLD + color.YELLOW + '| POINT ' + str(names[i]) + ' ' + str(points[i]) + ' | ' + str(Q[i]) + ' |' + color.END)
    return QPoints


def init_trajectory(trajectory):
    """ The trajectory message is initialized here when first called.
    The joints must be changed every time a new robot is connected to the system,
    according to how many joints it has. TO DO: loads this info from userdata given at launch? """

    trajectory.header.seq = 1
    trajectory.header.stamp.secs = 0
    trajectory.header.stamp.nsecs = 0
    trajectory.header.frame_id = 'robot_trajectory'
    trajectory.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

def write_trajectory(trajectory, positions, velocities, accelerations, effort):
    """ Method to create a JointTrajectory message ready to be sent to the driver node.
    The sequence number and time stamp are calculated based on what was the previous message sent.
    Each value of positions, velocity and so on refers to a specific joint in the same list position.
    Note that positions etc must be given to the function as lists: [[0,0,0,0,0,0], [1,1,1,1,1,1], ...] """

    for i in range(0,len(positions)):
        points = JointTrajectoryPoint()
        points.positions = positions[i]
        points.velocities = velocities[i]
        points.accelerations = accelerations[i]
        points.effort = effort[i]

        trajectory.points.append(points)

def send_trajectory(trajectory, pub):
    """ Method to send the trajectory with the current time in the timestamp. """

    trajectory.header.stamp = rospy.Time.now()
    pub.publish(trajectory)
    rospy.loginfo(color.BOLD + color.GREEN + '-- TRAJECTORY SENT --' + color.END)

def move_to_point(pub, trajectory, P, V, A, E):
    """ Function to fill in the trajectory message according to the action.
    In this case, the action is to move the robot to a specific point.
    The point P can be set choosing from the list of available points or in a static way.
    Remember the structure of P, V, A, and E: these are list of lists, each object contains
    as many items as the number of joints, and the list contains as many objects as the
    subsequent messages to be sent. I. e. for a 6 joints robot P = [[1,2,3,4,5,6]] """

    rospy.loginfo(color.BOLD + color.YELLOW + '-- MOVE ROBOT TO POSITION --' + color.END)
    # fills in the message
    write_trajectory(trajectory, P, V, A, E)
    # sends it
    send_trajectory(trajectory, pub)
    # after calling this function, the code must check if the goal has been reached
    # by calling the subscriber and checking the self.goal variable. If goal is true
    # then other actions can be performed afterward

def control_gripper(command):
    """ Function to open the end effector aka the gripper or closing it according to the command given.
    If command == 1 opens gripper, if command == -1 closes gripper. """

    # where is the gripper defined?

    if command == 1:
        # opens gripper
        rospy.loginfo(color.BOLD + color.YELLOW + '-- OPEN GRIPPER --' + color.END)
    else:
        # closes gripper
        rospy.loginfo(color.BOLD + color.YELLOW + '-- CLOSE GRIPPER --' + color.END)

def screw(pub):
    """ To screw something usually the last-1 joint is the one to move.
    The function selects the joint and make it revolve around its axis to screw an object handled by the gripper.
    It assumes that the robot is already in the right position and the only thing left to do
    is to screw the object in place! """

    rospy.loginfo(color.BOLD + color.YELLOW + '-- SCREW OBJECT --' + color.END)
