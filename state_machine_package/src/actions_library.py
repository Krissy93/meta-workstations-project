#!/usr/bin/env python

import rospy
from robot import *
from utils import *
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import rospkg

PKG_PATH = rospkg.RosPack().get_path('state_machine_package')

def action_list():
    """ Prints all the available actions present in this file. """

    rospy.loginfo(color.BOLD + color.PURPLE + '|-------------------|' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| AVAILABLE ACTIONS |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 1: MOVE TO POINT  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 2: OPEN GRIPPER   |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '| 3: CLOSE GRIPPER  |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '|-2: EXIT           |' + color.END)
    rospy.loginfo(color.BOLD + color.PURPLE + '|-------------------|' + color.END)

    actions = ['MOVE TO POINT', 'OPEN GRIPPER', 'CLOSE GRIPPER']

    return actions

def init_points2():
    """ Loads the XYZ points file and calculates the joints coordinates according
    to the robot-dependant inverse kinematic function. This function is defined in robot.py """

    # loads the points file
    file = load_txt(PKG_PATH + '/data/SPoints.txt')
    names, points = load_points(file)
    # checks for duplicates. Just a safety measure
    names, points = checkifduplicates(names, points)
    # performs the points elaboration in order to obtain the QQ joints coordinates of the XYZ positions.
    # the function calls the corresponding inverse kinematics function of the robot
    Q = elaborateKinMulti(points)
    # writes them in the corresponding file. It is not an append but a complete rewriting of the contents
    QPoints = write_txt(PKG_PATH + '/data/QPoints.txt', names, Q)
    # debug
    print(color.BOLD + color.PURPLE + '-- JOINTS COORDINATES RECALCULATED!--' + color.END)
    for i in range(0,len(points)):
        print(color.BOLD + color.YELLOW + '| POINT ' + str(names[i]) + ' ' + str(points[i]) + ' | ' + str(QPoints[i]) + ' |' + color.END)
    return QPoints

def init_points():
    """ Loads the joint points file into memory """

    # loads the points file
    file = load_txt(PKG_PATH + '/data/QPoints.txt')
    names, points = load_points(file)

    rospy.loginfo(color.BOLD + color.PURPLE + '-- JOINTS COORDINATES: --' + color.END)
    for i in range(0,len(points)):
        rospy.loginfo(color.BOLD + color.YELLOW + '| POINT ' + str(names[i]) + ' ' + str(points[i]) + color.END)
    return points

def init_trajectory(trajectory, empty):
    """ The trajectory message is initialized here when first called.
    The joints must be changed every time a new robot is connected to the system,
    according to how many joints it has.
    TO DO: loads this info from userdata given at launch? """

    trajectory.header.seq = 1
    #trajectory.header.stamp.secs = 0
    #trajectory.header.stamp.nsecs = 0
    #trajectory.header.frame_id = ''
    #trajectory.joint_names = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    trajectory.name = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint', 'ur10_elbow_joint', 'ur10_wrist_1_joint', 'ur10_wrist_2_joint', 'ur10_wrist_3_joint']
    #write_trajectory(trajectory, empty, empty, empty, empty)
    #print(trajectory)

def write_trajectory2(trajectory, positions, velocities, accelerations, effort):
    """ Method to create a JointTrajectory message ready to be sent to the driver node.
    The sequence number and time stamp are calculated based on what was the previous message sent.
    Each value of positions, velocity and so on refers to a specific joint in the same list position.
    Note that positions etc must be given to the function as lists: [[0,0,0,0,0,0], [1,1,1,1,1,1], ...]

    OLD VERSION"""

    # needed to set a single point in the right format.
    positions = [positions]
    velocities = [velocities]
    accelerations = [accelerations]
    effort = [effort]
    # needed to empty the previous portion of the message which contains the point
    trajectory.points = []

    for i in range(0,len(positions)):
        points = JointTrajectoryPoint()
        points.positions = positions[i]
        points.velocities = velocities[i]
        points.accelerations = accelerations[i]
        points.effort = effort[i]

        trajectory.points.append(points)
    print(trajectory)

def write_trajectory(trajectory, positions, velocities, effort):
    """ Method to create a JointTrajectory message ready to be sent to the driver node.
    The sequence number and time stamp are calculated based on what was the previous message sent.
    Each value of positions, velocity and so on refers to a specific joint in the same list position.
    Note that positions etc must be given to the function as lists: [[0,0,0,0,0,0], [1,1,1,1,1,1], ...] """

    # needed to set a single point in the right format.
    trajectory.position = []
    trajectory.velocity = []
    trajectory.effort = []

    for i in range(0,len(positions)):
        trajectory.position.append(float(positions[i]))
        trajectory.velocity.append(float(velocities[i]))
        trajectory.effort.append(float(effort[i]))

    print(trajectory)

def send_trajectory(trajectory, pub):
    """ Method to send the trajectory with the current time in the timestamp. """

    trajectory.header.stamp = rospy.Time.now()
    pub.publish(trajectory)
    rospy.loginfo(color.BOLD + color.GREEN + '-- TRAJECTORY SENT --' + color.END)

def move_to_point(pub, trajectory, P, V, E):
    """ Function to fill in the trajectory message according to the action.
    In this case, the action is to move the robot to a specific point.
    The point P can be set choosing from the list of available points or in a static way.
    Remember the structure of P, V, A, and E: these are list of lists, each object contains
    as many items as the number of joints, and the list contains as many objects as the
    subsequent messages to be sent. I. e. for a 6 joints robot P = [[1,2,3,4,5,6]] """

    rospy.loginfo(color.BOLD + color.YELLOW + '-- MOVE ROBOT TO POSITION --' + color.END)
    # fills in the message
    write_trajectory(trajectory, P, V, E)
    # sends it
    send_trajectory(trajectory, pub)
    # after calling this function, the code must check if the goal has been reached
    # by calling the subscriber and checking the self.goal variable. If goal is true
    # then other actions can be performed afterward

def servCallback(request):
    if request.success == True:
        rospy.loginfo(color.BOLD + color.GREEN + '-- SUCCESS --' + color.END)
    return request.success


def control_gripper(command):
    """ Function to open the end effector aka the gripper or closing it according to the command given.
    If command == 1 opens gripper, if command == -1 closes gripper. """

    # where is the gripper defined?

    grip = SetBool()
    if command == 1:
        # opens gripper
        grip = False
        rospy.loginfo(color.BOLD + color.YELLOW + '-- OPEN GRIPPER --' + color.END)
        #print('OK')
    else:
        # closes gripper
        grip = True
        rospy.loginfo(color.BOLD + color.YELLOW + '-- CLOSE GRIPPER --' + color.END)
        #print('NOK')

    rospy.wait_for_service('/gripper/grasp')
    try:
        grip_service = rospy.ServiceProxy('/gripper/grasp', SetBool)
        response = grip_service(grip)
        grip_service.close()
        return response
    except rospy.ServiceException:
        rospy.logerror('Service call failed')
