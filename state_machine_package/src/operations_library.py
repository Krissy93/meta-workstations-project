#!/usr/bin/env python

import rospy
from utils import *
import os
import shutil
import codecs
import time
import glob

def translate_operation(pkg_path, action_list):
    ''' Used to translate actions saved in a list.
    The list is created during the pick and place state execution.
    Every action taken is saved in a list of tuples (action number, point),
    if no point is assigned to the action, the point is saved as "None".

    If the user wants to save the sequence of actions in an operation file,
    this function reads from the Operations directory the number of operation files,
    then assigns to the new file a base name and the current date.
    '''
    BASENAME = '/operations/operation_'
    INDENT = '    '
    # finds the directory "operations" and creates the base name of the new file.
    # pkg_path = rospkg.RosPack().get_path('state_machine_package')

    # creates name of new file according to the current date
    date = time.strftime(r"%d-%m-%Y-%H:%M:%S", time.localtime())
    filename = BASENAME + date + '.py'
    # copies empty.py file in the new directory with the new name
    shutil.copy(pkg_path + '/data/empty.py', pkg_path + filename)

    with codecs.open(pkg_path + filename, "a") as file:
        file.write('\n\n' + INDENT + INDENT + '# START OF AUTO-WRITTEN PORTION')

        # append to file portions of txt according to the saved action
        for i in range(0,len(action_list)):
            current_action = action_list[i][0]
            if current_action == 2:
                # writes open gripper action
                file.write('\n\n' + INDENT + INDENT + 'control_gripper(1)')
            elif current_action == 3:
                # close gripper action
                file.write('\n\n' + INDENT + INDENT + 'control_gripper(-1)')
            elif current_action == 1:
                # write the move to point action according to the point saved
                # write: move_to_point(pub, action_list[i][1], empty, empty)
                file.write('\n\n' + INDENT + INDENT + 'move_to_point(self.pub, self.trajectory, ' + str(action_list[i][1]) + ', self.empty, self.empty)')

            # before going further, writes the feedback while loop
            if current_action != 2 and current_action != 3:
                # until goal not reached, checks the feedback topic
                file.write('\n\n' + INDENT + INDENT + 'self.goal = False')
                file.write('\n' + INDENT + INDENT + 'while (self.goal == False):')
                file.write('\n' + INDENT + INDENT + INDENT + 'self.trajsub = rospy.Subscriber(ROBOT_FEEDBACK, JointState, self.feedbackCallback, queue_size = 1)')
                file.write('\n' + INDENT + INDENT + INDENT + 'if self.goal == True:')
                file.write('\n' + INDENT + INDENT + INDENT + INDENT + 'rospy.loginfo(color.BOLD + color.YELLOW + \'-- POSITION REACHED --\' + color.END)')
                file.write('\n' + INDENT + INDENT + INDENT + INDENT + 'break')

        file.write('\n\n' + '# OBJECT THAT CONTAINS THE OPERATION AS A LIST OF ACTION CALLS')
        file.write('\n' + 'class OpObject():')
        file.write('\n' + INDENT + 'def __init__(self):')
        file.write('\n' + INDENT + INDENT + 'self.list = ' + str(action_list))

    rospy.loginfo(color.BOLD + color.GREEN + '-- OPERATION FILE WRITTEN IN: ' + filename + ' --' + color.END)

def init_operations(pkg_path):
    ''' Loads all the files in the operations folder and returns the list of files
    with their corresponding path. By doing this, the user can select each file using the
    filename as handler and execute it '''

    BASENAME = '/operations/operation_'
    # finds the directory "operations" and creates the base name of the new file.

    # creates a list of file paths contained in the operations folder of the package.
    files = filter(os.path.isfile, glob.glob(pkg_path + '/operations/' + "*.py"))
    # orders them according to modification date. Last modifified items are at the bottom of the list
    files.sort(key=lambda x: os.path.getmtime(x))

    return files

#translate_operation('/home/optolab/smach_ws/src/state_machine_package', [(2,None),(1,[1,1,1,1,1,1]),(3,None)])
