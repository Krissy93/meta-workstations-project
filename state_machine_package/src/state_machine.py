#!/usr/bin/env python

import rospy
import smach
import smach_ros
import sys
import time
import subprocess
from state_machine_package.msg import commandRequest, commandResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import threading
import traceback
from utils import *
from actions_library import *

########### STATE MACHINE BASIC CLASSES DEFINITION ###########

class State(smach.State):
    """ This is the main parents class for creation of
    states. Every child state inherit the common traits. """

    def __init__(self, transitions, input, output):
        """ Constructor for the state.
        Outcomes are a list of strings like: ['tState0','tState1']
        nextstate represents the number of the next state, needed to perform the transition
        command represent the numerical value attached to a command read by the gestures node
        statename is a debug value, contains the name of the current state
        request is initialized as an empty request message
        the subscriber is called only when needed, so it is initialized at zero
        the publisher is initialized at startup """

        smach.State.__init__(self, outcomes=transitions, input_keys=input, output_keys=output)
        self.nextstate = -1
        self.command = -1
        self.goal = False
        self.last_position = None
        self.statename = 'None'
        self.sub = 0
        self.trajsub = 0
        self.pub = rospy.Publisher('/command_request', commandRequest, queue_size = 1)
        self.trajpub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size = 1, latch=True)
        self.request = commandRequest()
        self.trajectory = JointTrajectory()
        # basic empty vector useful for initialization of velocities accelerations and effort
        # it is a vector of 10 elemets, which is the maximum. It is okay because messages
        # are created based on the position vector, which is robot dependant
        self.empty = [0,0,0,0,0,0,0,0,0,0]

    def stateCallback(self, msg):
        """ The structure of the request is:
        int64 request_number
        string request_type

        The callback needs to check if the number of the response received in msg
        is the same as the request number sent in /command_request.
        Depending on the type of request (interface or change state), the command is
        saved in the appropriate variable. """

        if msg.request_number == self.request.request_number:
            new = msg.command
            if msg.request_type == 'interface_command':
                self.command = new
            elif msg.request_type == 'change_state':
                self.nextstate = new

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
        #if (msg.position <= self.trajectory.positions - 0.01) and (msg.position >= self.trajectory.positions + 0.01):
        #print(self.trajectory.points[0].positions)
        if (msg.position[0] == 888) and (msg.position[1] >= self.trajectory.points[0].positions[1] - 0.01) and (msg.position[1] <= self.trajectory.points[0].positions[1] + 0.01):
            # if the two positions are almost equal, it means that the robot
            # has reached the goal of the last message. Sets the goal to true
            self.goal = True
        else:
            self.goal = False
        # regardless of the goal, it updates the position according to what is read on the topic
        self.last_position = msg.position

    def reset(self):
        """ Debug method that resets variables. """

        self.nextstate = -1
        self.command = -1
        self.goal = False

    def checkvalues(self):
        """ Debug method that prints current variables values. """

        print('Nextstate: ' + str(self.nextstate))
        print('Command: ' + str(self.command))

    def update(self, type):
        """ Method to update the request depending on what is specified by the state.
        According to the type of action needed, the message is compiled and sent as a request. """

        self.reset()
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = type
        self.pub.publish(self.request)

    def confirm(self):
        """ Function that checks if the user confirms, cancels or ask to go back home.
        It's called after a transition command has been given to the state machine,
        so it's common between the states and it's inherited from the parent class State. """

        self.command = -1
        #self.checkvalues()
        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ' STATE: CONFIRM?]' + color.END)
        # updates request number and type, since it waits for a confirmation command to confirm the transition
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = 'interface_command'
        # request sent: waits for confirm/cancel/delete
        self.pub.publish(self.request)

        while self.command == -1:
            # reads the response topic and waits for the correct response to appear
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.command == -3:
                # cancels the inserted command. After this command, the state machine exits the "confirm"
                # method but still waits inside the changestate method.
                rospy.loginfo(color.BOLD + color.YELLOW + '-- CANCELLED --' + color.END)
                break
            elif self.command == 0:
                # confirms the inserted command. Exits the "confirm" method and in the
                # changestate method performs the corresponding action.
                rospy.loginfo(color.BOLD + color.GREEN + '-- CONFIRMED --' + color.END)
                break
            elif self.command == -2:
                # exits the state, returning to the preceding one.
                rospy.loginfo(color.BOLD + color.RED + '-- EXIT STATE --' + color.END)
                break
            else:
                # waits
                self.command = -1


class InstructionState(State):
    """ A type of first grade child state which is the one used for states when
    an instruction has to be composed. It has a second method and another flag. """

    def __init__(self, transitions, input, output):
        """ It also has the instruction variable, since it can read commands and instructions. """

        State.__init__(self, transitions, input, output)
        self.instruction = -1

    def instructionCallback(self, msg):
        """ The structure of the request is:
        int64 request_number
        string request_type

        The callback needs to check if the number of the response received in msg
        is the same as the request number sent in /command_request.
        Depending on the type of request (interface, change state or instruction), the command is
        saved in the appropriate variable. """

        if msg.request_number == self.request.request_number:
            new = msg.command
            if msg.request_type == 'interface_command':
                self.command = new
            elif msg.request_type == 'change_state':
                self.nextstate = new
            elif msg.request_type == 'instruction_command' or msg.request_type == 'jog_command':
                self.instruction = new

    def reset(self):
        """ Debug method that resets variables. """

        self.command = -1
        self.nextstate = -1
        self.goal = False
        self.instruction = -1

    def checkvalues(self):
        """ Debug method that prints current variables values. """

        print('Nextstate: ' + str(self.nextstate))
        print('Command: ' + str(self.command))
        print('Instruction: ' + str(self.instruction))

    def update(self, type):
        """ Method to update the request depending on what is specified by the state.
        According to the type of action needed, the message is compiled and sent as a request. """

        self.reset()
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = type
        self.pub.publish(self.request)

    def confirmInstruction(self, name):
        """ Function that checks if the user confirms, cancels or ask to go back home.
        It's called after an instruction command has been given to the state machine. """

        self.command = -1
        #self.checkvalues()
        rospy.loginfo(color.BOLD + color.CYAN + '[RECEIVED COMMAND: ' + str(name) + '. CONFIRM?]' + color.END)
        # updates request number and type, since it waits for a confirmation command to confirm the instruction
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = 'interface_command'
        # request sent: waits for confirm/cancel/delete
        self.pub.publish(self.request)

        while self.command == -1:
            # reads the response topic and waits for the correct response to appear
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.command == -3:
                # cancels the inserted command. After this command, the state machine exits the "confirmInstruction"
                # method but still waits inside the changestate method.
                rospy.loginfo(color.BOLD + color.YELLOW + '-- CANCELLED --' + color.END)
                break
            elif self.command == 0:
                # confirms the inserted command. Exits the "confirm" method and in the
                # changestate method performs the corresponding action.
                rospy.loginfo(color.BOLD + color.GREEN + '-- CONFIRMED --' + color.END)
                break
            elif self.command == -2:
                # exits the state, returning to the preceding one.
                rospy.loginfo(color.BOLD + color.RED + '-- EXIT STATE --' + color.END)
                break
            else:
                # waits
                self.command = -1

    def getInstruction(self):
        """ Method to create the instruction.
        It can only read NUMBERS and the GO instruction """

        self.instruction = -1
        #self.checkvalues()
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = 'instruction_command'
        self.pub.publish(self.request)

        while self.instruction == -1:
            # it can read every numerical instruction. Numbers are defined between 1000 and 1009
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)

            # if it reads a number or the GO instruction then it can perform some actions
            if (self.instruction > 999) or (self.instruction == 0):
                # change name accordingly
                if self.instruction == 0:
                    name = '[GO]'
                else:
                    name = '[NUMBER ' + str(self.instruction - 1000) + ']'

                #self.checkvalues()
                # asks the user if the inserted instruction is correct
                self.confirmInstruction(name)
                # after this call, the system has in memory "cancel", "confirm" or "exit"
                #self.checkvalues()

                if self.command == 0:
                    # instruction has been confirmed, thus command = 0 and instruction = value
                    rospy.loginfo(color.BOLD + color.GREEN + '-- COMMAND ' + str(name) + ' SAVED --' + color.END)
                    break
                elif self.command == -2:
                    # exits state requested, the next state is changed accordingly
                    self.nextstate = -2
                    break
                else:
                    # deleted instruction, the state machine has to re-send a request for a new instruction
                    # to substitute the old one. It needs to call the updating method to do so,
                    # and reset the instruction to keep staying inside the while loop
                    rospy.loginfo(color.BOLD + color.YELLOW + '-- PLEASE RE-ENTER INSTRUCTION --' + color.END)
                    self.update('instruction_command')

            elif self.instruction == -2:
                # if it has not read a number or GO but an exit command instead, it just exits updating nextstate
                self.nextstate = -2
                break

            else:
                # waits
                self.instruction = -1

    def getJog(self):
        """ Method to create the jog instruction type.
        It can only read JOG COMMANDS, PAUSE, GO and EXIT.
        WARNING: CURRENTLY UNUSED, JOG COMMANDS ARE DEFINED IN THE RUN JOG STATE """

        self.instruction = -1
        self.request.request_number = self.request.request_number + 1
        self.request.request_type = 'jog_command'
        self.pub.publish(self.request)

        while self.instruction == -1:
            # it can read every numerical instruction. Jog commands follow a different rule,
            # these are numbers > 10000 defined as 10[0joint][0/1/-1] to represent a jog command,
            # which joint to select and if the command is to stop, increase or decrease position.

            self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)

            if (self.instruction > 10000):
                # if it reads a jog instruction prints a debug info
                rospy.loginfo(color.BOLD + color.GREEN + '-- JOG --' + color.END)
                break

            elif self.instruction == -4:
                # paused execution
                rospy.loginfo(color.BOLD + color.YELLOW + '-- PAUSED --' + color.END)
                break

            elif self.instruction == -2:
                # exits the state
                self.nextstate = -2
                break

            else:
                # waits
                self.instruction = -1

########### STATE MACHINE STATES DEFINITION ###########

class ReadyState(State):
    """ Initial state when the program is first launched.
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tHome','texit'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)
        self.statename = 'INITIAL STATE'

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|           COMMAND LIST          |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: GO TO HOME STATE             |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: CLOSE COMMUNICATION AND EXIT |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'INITIAL STATE'
        self.reset()
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input + 1

        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ': READY TO COMMUNICATE]' + color.END)
        self.listCommands()
        # from this state the only action possible is to change the state or close communication,
        # thus the request type is initialized as "change state"
        self.request.request_type = 'change_state'
        # sleep is needed to correctly send the request in the topic,
        # since at startup it may not be already up if the user is fast and already sends a command
        time.sleep(1)
        #print(self.request)
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == 0:
                self.statename = 'HOME'
                self.confirm()
                # asks the user to confirm the state read from the response message.

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'ROOT'
                    return 'tHome'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == -2:
                # if the command is to exit the state, it needs to exit the while loop here
                break

            else:
                # just waits to read something useful from the subscriber
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Exits the program and updates variables (useful for debug)
        rospy.loginfo(color.BOLD + color.RED + '-- EXITING PROGRAM --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'ROOT'
        #self.request_preempt() ????
        return 'texit'


class Home(State):
    """ Home state. From here, the user can choose what to do, transitioning to the
    different states. New functionalities must be connected to this state to be accessed!
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tState0','tPointsDef', 'tLoop', 'tPickPlace', 'tJog', 'tSetVel'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)
        self.statename = 'HOME STATE'

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|           COMMAND LIST           |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-5: GO TO POINTS DEFINITION STATE |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 1: GO TO LOOP DEFINITION STATE   |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 2: GO TO PICK AND PLACE STATE    |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 3: GO TO JOG STATE               |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 4: GO TO SET MOTOR SPEED STATE   |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: CLOSE COMMUNICATION (STATE 0) |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'HOME STATE'
        self.reset()
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input + 1
        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ': WHAT DO YOU WANT TO DO?]' + color.END)
        self.listCommands()
        # from this state the only action available is to change state or exit,
        # thus the command type is a change state one.
        self.request.request_type = 'change_state'
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == -5:
                # chooses the point definition state
                self.statename = 'POINTS DEFINITION'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tPointsDef'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == 1:
                # chooses the loop state
                self.statename = 'LOOP'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tLoop'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == 2:
                # chooses the pick and place state
                self.statename = 'PICK AND PLACE'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tPickPlace'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == 3:
                # chooses the jog state
                self.statename = 'JOG'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tJog'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == 4:
                # chooses to change robot movement speed
                self.statename = 'SET ROBOT MOVEMENT SPEED'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tSetVel'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate = -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate == -2:
                # if the command is to exit the state, it needs to exit the while loop here
                break

            else:
                # just waits to read something useful from the subscriber
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Returns to the ReadyState, basically closing the communication and updates variables (useful for debug)
        self.statename = 'STATE 0'
        rospy.loginfo(color.BOLD + color.RED + '-- CLOSING COMMUNICATION, RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'HOME STATE'
        return 'tState0'

########### POINTS DEFINITION ###########

class PointsDef(State):
    """ Points definition state. The only thing that happens here is that the corresponding
    file is opened and ready to be modified (keyboard inputs, not gestures).
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tHome'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        self.statename = 'POINTS DEFINITION STATE'
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 1: ' + self.statename + ']' + color.END)

        # it uses a bunch of functions imported from utils.
        rospy.loginfo(color.BOLD + color.PURPLE + '-- OPENING XYZ POINTS FILE --' + color.END)
        # launches the gedit terminal command to open the file. It is statically defined here
        return_code = subprocess.call("gedit /home/ros/catkin_ws/src/state_machine_package/data/SPoints.txt", shell=True)
        rospy.loginfo(color.BOLD + color.PURPLE + '-- DONE! --' + color.END)
        # after modifiying, it loads the contents of the file into memory. S points are in XYZ coordinates
        QQ = init_points()

        # returns home automatically after these operations are concluded.
        self.statename = 'HOME STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'POINTS DEFINITION STATE'
        return 'tHome'

########### LOOP STATES ###########

'''NON USATO... RIMUOVERE'''
class Loop(State):
    """ Loop operation state. A trajectory is selected from the list of available trajectories
    and executed in a loop until a stop or pause command is not seen, or the loop ends.
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tHome', 'tLTraj', 'tLRun'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)
        self.statename = 'LOOP DEFINITION STATE'

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|        COMMAND LIST        |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-5: CHANGE TRAJECTORY FILE  |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: START A LOOP            |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: RETURN HOME             |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'LOOP DEFINITION STATE'
        self.reset()
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input + 1
        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 2: ' + self.statename + ']' + color.END)
        self.listCommands()
        # from this state the only action available is to change state or exit,
        # thus the command type is a change state one. This is because the real "action" states are the sub states
        self.request.request_type = 'change_state'
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == -5:
                # first sub state selected, the trajectory file can be changed easily
                self.statename = 'CHANGE TRAJECTORY FILE'
                self.confirm()
                # asks the user to confirm the selection
                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'LOOP DEFINITION STATE'
                    return 'tLTraj'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate == -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update(userdata, 'change_state')


            elif self.nextstate == 0:
                # chooses to start a loop
                self.statename = 'START LOOP'
                self.confirm()
                # asks the user to confirm the selection
                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'LOOP DEFINITION STATE'
                    return 'tLRun'
                elif self.command == -2:
                    # the command given asks to exit the state, in this case closing the communication.
                    self.nextstate == -2
                    break
                else:
                    # in this case the command has been deleted, thus it needs reprint the command list and wait
                    # for a new command to be insterted instead, after the updating.
                    self.listCommands()
                    self.update(userdata, 'change_state')


            elif self.nextstate == -2:
                # if the command is to exit the state, it needs to exit the while loop here
                break

            else:
                # just waits to read something useful from the subscriber
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Returns to the home state and updates variables (useful for debug)
        self.statename = 'HOME STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'LOOP DEFINITION STATE'
        return 'tHome'


'''NON USATO... RIMUOVERE'''
class LoopTrajectory(State):
    def __init__(self, transitions=['tLoop'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        self.statename = 'CHANGE TRAJECTORY FILE'
        rospy.loginfo(color.BOLD + color.CYAN + '[LOOP STATE 1: ' + self.statename + ']' + color.END)

        rospy.loginfo(color.BOLD + color.PURPLE + '-- OPENING TRAJECTORY FILE --' + color.END)
        # launches the gedit terminal command to open the file. It is statically defined here
        return_code = subprocess.call("gedit /home/ros/catkin_ws/src/state_machine_package/data/SPoints.txt", shell=True)
        rospy.loginfo(color.BOLD + color.PURPLE + '-- DONE! --' + color.END)
        ########################## EDIT RICHIESTO QUI
        # returns home automatically after these operations are concluded.
        self.statename = 'LOOP STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'CHANGE TRAJECTORY STATE'
        return 'tLoop'


'''TO DO: SISTEMARE COMPLETAMENTE'''
class LoopRun(InstructionState):
    ''' Loop state. The operator selects the operation to be executed from a list of
    available operations, loads it into memory then says to the robot how many times it
    must execute the loop. It can be paused or stopped in the time between the sending of packets.
    It inherits common InstructionState methods. '''

    def __init__(self, transitions=['tLoop'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        InstructionState.__init__(self, transitions, input, output)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'START CYCLE' #TO DO: DA INSERIRE IL COMANDO DI PAUSA!! POTREI FARLO ANDARE IN UN ALTRO STATO
        self.reset()
        # the request number do not need to be updated here
        self.request.request_number = userdata.input

        # loads the operations file. The file contains the names of the operations in the first column, and the paths in the second column
        operationfile = load_txt('/home/ros/catkin_ws/src/state_machine_package/data/Operations.txt')
        rospy.loginfo(color.BOLD + color.CYAN + 'CHOOSE EXISTING OPERATION FROM LIST:' + color.END)
        # prints the available operations from the file, so that the user can select one. The corresponding operation
        # is then loaded from its personal file which contains every detail of the execution
        for i in range(0,len(operationfile)):
            rospy.loginfo(color.BOLD + color.YELLOW + '| OP. ' + str(i) + ': ' + str(operationfile[i]) + ' |' + color.END)
            #TO DO: qui e' da sistemare perche' una operazione contiene una serie di punti e parametri che determinano la sua esecuzione da inizio e fine
            # nel file operations invece salvo solo gli header che determinano il nome del file di riferimento nel quale e' salvata l'operazione
            # e che verra' caricato di conseguenza quando selezionato

        # asks the user to select an operation, thus a numeric command + GO is needed
        self.instruction()

        if self.instruction > 999:
            # if a numerical command has been obtained, it is saved and used to select
            # the operation from the list. "traj" corresponds to the row number of the file.
            # the file contains at row traj in position 0 the name of the operation,
            # in position 1 the path of the corresponding trajectory definition file
            traj = self.instruction
            # asks the user how many times the operation must be executed
            rospy.loginfo(color.BOLD + color.CYAN + 'HOW MANY REPETITIONS?' + color.END)
            self.instruction()

            if self.instruction > 999:
                # another numerical command is needed here to select the number of repetitions
                # since numbers are defined as > 999, to obtain the single number we need to
                # subtract 1000. Remember that 999 = go and 1000+ are the numbers
                cycles = self.instruction - 1000
                rospy.loginfo(color.BOLD + color.CYAN + 'OK! READY TO SEND THIS INSTRUCTION?' + color.END)
                # plots the recap of the composed action obtained so far.
                rospy.loginfo(color.BOLD + color.YELLOW + 'OPERATION: ' + str(operationfile[traj][0]) + ', REPEAT FOR: ' + str(cycles) + ' TIMES.' + color.END)
                self.instruction()
                # asks the user to confirm the recap. if so, it loads the corresponding trajectory file and starts the execution

                if self.instruction == 0:
                    # sets the goal to false
                    goal_reached = False
                    trajectory_file = load_txt(str(operationfile[traj][1]))
                    rospy.loginfo(color.BOLD + color.GREEN + '-- SENDING COMMAND TO ROBOT --' + color.END)
                    # TO DO: se il file della traiettoria e' fatto bene dovrebbe contenere gia' di suo le posizioni
                    # e i vari step che il robot deve eseguire, quindi basta mandarle in un loop... o forse basta in blocco? chiedi a paga
                    # TO DO: INVIA IL COMANDO
                    # POI ESCE E TORNA IN CYCLE QUANDO MI ARRIVA LA RISPOSTA DAL DRIVER (SONO ARRIVATO)?
                    # TO DO: pubblica il comando traj + cycles per il driver
                    # devo chiamare la funzione di cinematica?
                    # a questo punto aspetto che il driver mi dica che ha finito il ciclo;
                    # e continuo a leggere su commands in un loop in attesa del comando di pausa o di stop

                    # sends the request to be able to wait for a stop or pause command
                    self.request.request_number = self.request.request_number + 1
                    self.request.request_type = 'change_state'
                    self.pub.publish(self.request)

                    # checks the topic until the execution is not finished
                    while goal_reached == False:
                        self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)
                        if self.nextstate == -2:
                            # if a stop command is given, the robot must stop the execution and return in the ReadyState
                            #TO DO: pubblica il messaggio di stop per il robot e torna in stato 0
                            rospy.loginfo(color.BOLD + color.RED + '-- ROBOT STOPPED --' + color.END)
                            self.statename = 'INITIAL STATE'
                            rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
                            userdata.output = self.request.request_number
                            userdata.oldstate = 'START LOOP STATE'
                            return 'tState0'
                        elif self.nextstate == -4:
                            # if a pause command is given, the robot must stop the execution until a go command is not given afterwards.
                            # it needs to stay inside the while loop until this happens!
                            # TO DO: manda un comando di stop al driver, ma poi rimane nel while del goal perche' non c'e' goal reached
                            rospy.loginfo(color.BOLD + color.YELLOW + '-- EXECUTION PAUSED --' + color.END)

                            # resets some variables, cannot use the reset method here
                            self.command = -1
                            self.instruction = -1
                            # asks for a new command, in this case the go command to resume the operation
                            self.request.request_number = self.request.request_number + 1
                            self.request.request_type = 'instruction_command'
                            self.pub.publish(self.request)
                            # this while continues until a "GO" command is not given and confirmed (instruction and command both = 0)
                            while not (self.instruction == 0 and self.command == 0):
                                self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)
                                if self.instruction == 0:
                                    # if it reads a "go" asks for a confirm
                                    self.confirmInstruction('[GO]')
                                    # exits the while loop only if after this call command is == 0, otherwhise it keeps waiting
                                    if self.command != 0:
                                        rospy.loginfo(color.BOLD + color.RED + '-- NOT CONFIRMED, STILL PAUSED --' + color.END)
                                    else:
                                        break
                                if self.instruction == -2 or self.command == -2:
                                    # if somehow the exit command is given, it exits the state
                                    self.statename = 'LOOP STATE'
                                    rospy.loginfo(color.BOLD + color.RED + '-- EXECUTION CANCELLED, RETURNING TO ' + self.statename + ' --' + color.END)
                                    userdata.output = self.request.request_number
                                    userdata.oldstate = 'START LOOP STATE'
                                    return 'tLoop'

                            # if the program reaches this point, it means that it has exited the pause while loop, thus the execution can continue.
                            # remember that we are still in the main while loop, which ends only if the goal is reached; thus after this loginfo
                            # it simply restart the checks of the main while loop and keeps sending the trajectory packets to the robot
                            rospy.loginfo(color.BOLD + color.YELLOW + '-- CONTINUE EXECUTION --' + color.END)
                        elif goal_reached == True: #GOAL REACHED, me lo pubblica il driver come numero se finisce l'esecuzione
                            rospy.loginfo(color.BOLD + color.GREEN + '-- DONE! --' + color.END)
                        else: #significa che non legge comandi di interrupt vari quindi puo' continuare a inviare il resto dei pacchetti
                            # TO DO: continua l'invio (come aveva fatto rossano?)
                            dummy = 1 #istruzione a caso stupida per riempire senno' da' errore

                    self.statename = 'LOOP STATE'
                    rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'START LOOP STATE'
                    return 'tLoop'
                else: #e' uscito -2
                    rospy.loginfo(color.BOLD + color.RED + '-- OPERATION CANCELLED --' + color.END)
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'START LOOP STATE'
                    return 'tLoop'


            else: # e' uscito -2
                # annulla il comando e torna al main dello stato
                rospy.loginfo(color.BOLD + color.RED + '-- OPERATION CANCELLED --' + color.END)
                userdata.output = self.request.request_number
                userdata.oldstate = 'START CYCLE STATE'
                return 'tLoop'

        else: # e' uscito -2
            # annulla il comando e torna al main dello stato
            rospy.loginfo(color.BOLD + color.RED + '-- OPERATION CANCELLED --' + color.END)
            userdata.output = self.request.request_number
            userdata.oldstate = 'START CYCLE STATE'
            return 'tLoop'

########### PICK AND PLACE STATE ###########

'''CONTROLLA SE E' OK'''
class PickPlace(InstructionState):
    """ Pick and Place state. Allows the user to send to the robot some actions
    interactively, meaning that if no exit gesture is seen the robot waits for the next
    movement command. It inherits common traits from the parent InstructionState class. """

    def __init__(self, transitions=['tHome'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        InstructionState.__init__(self, transitions, input, output)

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|  COMMAND LIST  |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: RETURN HOME |' + color.END)

    def select_point_from_list(self):
        # resets variables
        self.reset()
        P = None
        # loads the points file in joints coordinates
        file = load_txt('/home/ros/catkin_ws/src/state_machine_package/data/QPoints.txt')
        names, points = load_points(file)
        rospy.loginfo(color.BOLD + color.PURPLE + '-- SELECT POINT FROM LIST --' + color.END)
        for i in range(0,len(names)):
            rospy.loginfo(color.BOLD + color.YELLOW + '| POINT ' + str(names[i]) + ': ' + str(points[i]) + ' |' + color.END)

        # updates and sends the request
        self.update('instruction_command')

        while self.instruction == -1:
            # waits for the correct instruction according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)

            # checks if number is in point list, starts from 1
            if (int(self.instruction) >= 1) and (int(self.instruction) <= len(points)):
                # selects the point
                P = points[int(self.instruction - 1)]
                N = names[int(self.instruction - 1)]

                # asks for a confirm
                name = 'POINT SELECTION OF POINT ' + str(N) + ': ' + str(P)
                self.confirmInstruction(name)
                if self.command == 0:
                    # confirmed point
                    break

                elif self.command == -2:
                    # exit command
                    P = None
                    self.nextstate = -2
                    break

                else:
                    # waits
                    rospy.loginfo(color.BOLD + color.YELLOW + '-- PLEASE RE-ENTER INSTRUCTION --' + color.END)
                    self.update('instruction_command')

        # after exiting the while loop, it returns the point P
        return P

    def select_action_from_list(self):
        # resets variables
        self.reset()
        # first prints the actions available
        actions = action_list()

        # updates and sends the request
        self.update('instruction_command')

        while self.instruction == -1:
            # waits for the correct instruction according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)

            # checks if number is in point list, starts from 1
            if (int(self.instruction) >= 1) and (int(self.instruction) <= len(actions)):
                # selects the action
                action = int(self.instruction)

                # asks for a confirm
                name = 'SELECTION OF ACTION: ' + str(actions[action - 1])
                self.confirmInstruction(name)
                if self.command == 0:
                    # confirmed action
                    break

                elif self.command == -2:
                    # exit command
                    action = None
                    self.nextstate = -2
                    break
                else:
                    # waits
                    rospy.loginfo(color.BOLD + color.YELLOW + '-- PLEASE RE-ENTER INSTRUCTION --' + color.END)
                    self.update('instruction_command')

        # after exiting the while loop, it returns the point P
        return action

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'PICK AND PLACE STATE'
        self.reset()
        init_trajectory(self.trajectory, self.empty)
        # initializes points just to be sure that everything is up to date
        #_ = init_points()
        # needed to correctly set the request number when entering the state
        #self.request.request_number = userdata.input + 1
        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 3: ' + self.statename + ']' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + 'READY TO RECEIVE INSTRUCTION!' + color.END)
        self.listCommands()
        # "change state" here means return home or start the pick and place operation mode
        #self.request.request_type = 'change_state'
        #self.pub.publish(self.request)

        while (self.nextstate == -1):
            P = None
            #self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)
            # asks the user which action it wants. exit state is translated as -2.

            # selects the action from the list
            action = self.select_action_from_list()
            # asks for a confirm
            if self.command == 0:
                # confirmed action! Selects the corresponding one from cases.
                # logs are printed inside the corresponding action functions
                # TO DO: HOW TO CONTROL GRIPPER?
                if (action == 2):
                    # open gripper action
                    control_gripper(1)
                elif (action == 3):
                    # close gripper action
                    control_gripper(-1)
                elif (action == 4):
                    # screw action, set to default
                    # TO DO: HOW TO DO THE SCREW COMMAND?
                    screw(self.trajpub)
                else:
                    # move to point action, needs a point to be executed
                    while (P == None):
                        # selects a point from the list, asking the user which one it wants
                        P = self.select_point_from_list()
                        # se ho P e non ho impostato nextstate a -2, procedo con l'azione
                        if self.command == 0:
                            # only if both the action and the point have been selected correctly
                            # then it calls the movement action
                            move_to_point(self.trajpub, self.trajectory, P, self.empty, self.empty, self.empty)

                        elif self.command == -2:
                            # exit command given
                            break

                        else:
                            # waits
                            P = None

                # here I have successfully sent the instruction.
                # I don't want to exit the state here, so I just resets everything
                # so I can start again, after asking the user if it wants to give a new command

                # waits for feedback
                # if feedback received, asks if new instruction must be sent or exit
                self.goal = False
                while (self.goal == False):
                    # until goal not reached, checks the feedback topic
                    self.trajsub = rospy.Subscriber('/joint_states', JointState, self.feedbackCallback, queue_size = 1)

                    if self.goal == True:
                        rospy.loginfo(color.BOLD + color.YELLOW + '-- POSITION REACHED --' + color.END)
                        rospy.loginfo(color.BOLD + color.CYAN + 'DO YOU WANT TO CONTINUE? (0) YES (-2) EXIT' + color.END)
                        # asks for the command
                        self.statename = 'CONTINUE?'
                        self.confirm()
                        if self.command == 0:
                            # continue, thus resets everything and stays inside main while loop
                            rospy.loginfo(color.BOLD + color.CYAN + 'READY TO RECEIVE NEW INSTRUCTION!' + color.END)
                            self.update('change_state')
                            # needed to exit this while loop but stays in the main one
                            break
                        elif self.command == -2:
                            # breaks and exits
                            self.nextstate = -2
                            break
                        else:
                            # stays in this while loop
                            self.goal = False
                    else:
                        # waits
                        self.goal = False


            elif self.command == -2:
                # exit command given
                break

            else:
                # waits
                self.nextstate = -1

        # does the following only when nextstate/command == -2, thus exiting from the state
        self.statename = 'HOME STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'PICK AND PLACE STATE'
        return 'tHome'

########### JOG STATES ###########

class Jog(State):
    """ Jog base state. The user can choose to start the Jog Mode or change the step size,
    since it affects how much each joint will move at each step.
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tHome', 'tJRun', 'tJStep'], input=['input'], output=['output', 'oldstate']):
        State.__init__(self, transitions, input, output)

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|       COMMAND LIST      |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: START JOG MODE       |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-5: CHANGE JOG STEP SIZE |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: RETURN HOME          |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'JOG MODE STATE'
        self.reset()
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input + 1
        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 4: ' + self.statename + ']' + color.END)
        self.listCommands()
        # from this state the user can move to two substates, thus the request type is a change state one
        self.request.request_type = 'change_state'
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == 0:
                # if the go command is given, the state machine moves to the start jog mode state
                self.statename = 'START JOG MODE'
                self.confirm()
                # asks the user to confirm the command
                if self.command == 0:
                    # if confirmed, it performs the transition
                    userdata.output = self.request.request_number
                    return 'tJRun'
                elif self.command == -2:
                    # if the exit command is given, it breaks the while loop and sets
                    # nextstate to -2
                    self.nextstate = -2
                    break
                else:
                    # if the command has been deleted, reprints the available commands
                    # and updates the request.
                    self.listCommands()
                    self.update(userdata, 'change_state')

            elif self.nextstate == -5:
                # collab gesture, it will change state
                userdata.oldstate = 'JOG MODE STATE'
                userdata.output = self.request.request_number
                return 'tJStep'

            elif self.nextstate == -2:
                # if exit, then breaks and executes final state code lines
                break

            else:
                # just waits for the subscriber to read something useful
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Returns to the home state and updates variables (useful for debug)
        self.statename = 'HOME STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'JOG MODE STATE'
        return 'tHome'


'''CONTROLLA SE E' OK'''
class JogRun(InstructionState):
    """ Run Jog State. This is the operative state where the user moves interactively
    the robot one joint at a time, telling it to increse or reduce its position.
    May be a good idea to print also the possible directions of each joint.
    It inherits common traits from the parent instruction State class. """

    def __init__(self, transitions=['tJog', 'tJStep'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        InstructionState.__init__(self, transitions, input, output)
        # also initializes two new variables for the state. The 0 joint does not exist and the 0 direction means stop.
        self.joint = 0
        self.direction = 0

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|                COMMAND LIST                |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: SAVE POINT TO FILE                      |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 10[0J][-1/0/1]: MOVE JOINT RIGHT/LEFT/STOP |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| -5: CHANGE JOG STEP SIZE                   |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| -2: RETURN TO JOG BASE STATE               |' + color.END)

    def execute(self, userdata):
        # initializes variables by calling the reset method
        self.statename = 'START JOG MODE'
        self.reset()
        init_trajectory(self.trajectory, self.empty)
        # updates the request number according to the old one
        self.request.request_number = userdata.input + 1
        # even if these have been initialized when the state has been built, it is
        # good practice to always initialize everything when calling the execute method of the state.
        # this is needed because the constructor is called once, when the program first starts, but the states can
        # be accessed multiple times during a single execution, thus it is important to have every variable initialized
        # when accessing the state, regardless how many times it has been accessed!
        self.joint = 0
        self.direction = 0
        # prints some debug info and the command list
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 4: ' + self.statename + ']' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + 'SELECT THE JOINT WITH ONE HAND AND TELL WITH THE OTHER IF IT HAS TO INCREMENT OR DECREMENT POSITION.' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + 'HAND POINTING RIGHT: INCREMENT POSITION (1), HAND POINTING LEFT: DECREMENT POSITION (-1), HAND OPEN (FIVE): STOPS (0)' + color.END)
        self.listCommands()
        # in this case the request type is directly the jog type, because this is an operative state
        self.request.request_type = 'jog_command'
        self.pub.publish(self.request)
        # TO DO: publish joints workspace

        while self.nextstate == -1:
            # updates directly the instruction not the "nextstate" value from the callback
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.instructionCallback, queue_size = 1)

            if (self.instruction == 0) or (self.instruction > 10000) or (self.instruction == -5):
                # the only available instructions are the GO (0) one, the JOG (10000) and the COLLAB (-5)
                if self.instruction == 0:
                    # in this case it saves the current position in a file
                    name = '[SAVE POINT]'
                    self.confirmInstruction(name)
                    # asks for a confirmation command
                    if self.command == 0:
                        # save point confirmed, it needs to open the S points file, append it, recalculate the Q points and close everything
                        # gets the position from the feedback topic of the robot controller driver
                        self.trajsub = rospy.Subscriber('/joint_states', JointState, self.feedbackCallback, queue_size = 1)
                        Q = self.last_position
                        # it is written as [0,0,0,0,0,0], already in joint state space
                        # loads the whole joints point list
                        QQ = load_txt('/home/ros/catkin_ws/src/state_machine_package/data/QPoints.txt')
                        # appends the new point to the list and checks for duplicates
                        QQ.append(Q)
                        QQ = checkifduplicates(QQ)
                        # performs the points elaboration in order to obtain the XYZ positions of the joints coordinates
                        # TO DO: use the direct kinematics function
                        SS = elaborateKinMulti(QQ)
                        # writes them in the corresponding file. It is not an append but a complete rewriting of the contents
                        SS = write_txt('/home/ros/catkin_ws/src/state_machine_package/data/SPoints.txt', Q)
                        rospy.loginfo(color.BOLD + color.GREEN + '-- POINT ' + str(S) + ' SAVED, DUPLICATES REMOVED --' + color.END)
                        # updates the request and resets variables, thus it remains in the while loop after this (not breaking it)
                        self.update('jog_command')

                    elif self.command == -2:
                        # exit command given, breaks the while and exits
                        self.nextstate = -2
                        break

                    else:
                        # cancel the insert point command. It needs to update the request to keep staying in the while loop
                        # and listen to other new commands.
                        self.update('jog_command')

                elif self.instruction == -5:
                    # in this case moves to the "change step size" state
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'START JOG MODE STATE'
                    return 'tJStep'

                else:
                    # if 0 nor -5 have been read, it means the subscriber got proper jog commands
                    # the jog instruction is defined as 10[0joint][1] right, 10[0giunto][0] stop, -10[0giunto][1] left
                    # to select the joint (assuming is not > 9): 10[0J][x]/10 = 10[0J].[x], discards the float since it is an int value
                    # then it just subtracts 1000 to obtain J
                    self.joint = self.instruction/10 - 1000
                    # after the joint selection we need to understand the directional command.
                    # this can be done checking the leftover of the division operation using %
                    if (abs(self.instruction) % 10) == 0:
                        # if this absolute value is 0 it means that [x] = [0] = stop
                        sdir = 'STOP'
                        self.direction = 0
                    elif (abs(self.instruction) % 10) == 1:
                        # if the absolute value is 1 it can mean 1 and -1 originally
                        if self.instruction > 0:
                            # checks the instruction as a whole: it is > 0? then it means +1 (right)
                            sdir = 'INCREMENT'
                            self.direction = 1
                        else:
                            # the whole instruction was negative, meaning -1 (left)
                            sdir = 'DECREMENT'
                            self.direction = -1
                    else:
                        # no valid number read, just resets and prints an error
                        self.joint = 0
                        self.direction = 0
                        rospy.loginfo(color.BOLD + color.RED + 'WRONG COMMAND' + color.END)

                    if (self.joint + self.direction != 0):
                        # if a valid command has been read (meaning that joint and direction are not 0)
                        # then we got a valid instruction and we can move the robot
                        # prints a log here to debug what is doing
                        rospy.loginfo(color.BOLD + color.YELLOW + '[JOINT ' + str(joint) + ': ' + sdir + 'POSITION]' + color.END)
                        # gets last position
                        self.trajsub = rospy.Subscriber('/joint_states', JointState, self.feedbackCallback, queue_size = 1)
                        # publish the movement command in the joint topic of the driver
                        positions = self.last_position + userdata.jogstep
                        write_trajectory(positions, self.empty, self.empty, self.empty)
                        send_trajectory()
                        # waits for feedback here
                        while (self.goal == False):
                            # until goal not reached, checks the feedback topic
                            self.trajsub = rospy.Subscriber('/joint_states', JointState, self.feedbackCallback, queue_size = 1)

                            if self.goal == True:
                                rospy.loginfo(color.BOLD + color.GREEN + '-- POSITION REACHED --' + color.END)
                                rospy.loginfo(color.BOLD + color.CYAN + 'DO YOU WANT TO CONTINUE? (0) YES (-2) EXIT' + color.END)
                                # asks for the command
                                self.statename = 'CONTINUE?'
                                self.confirm()
                                if self.command == 0:
                                    # continue, thus resets everything and stays inside main while loop
                                    rospy.loginfo(color.BOLD + color.GREEN + 'READY TO RECEIVE NEW JOG COMMAND!' + color.END)
                                    self.update('instruction_command')
                                    # needed to exit goal while
                                    break
                                elif self.command == -2:
                                    # breaks and exits
                                    self.nextstate = -2
                                    break
                                else:
                                    # stays in this while loop
                                    self.goal = False
                            else:
                                # waits
                                self.goal = False

            elif self.nextstate == -2:
                # operation closed, exits the state thus breaks the while
                break

            else:
                # waits until something useful is not read from the subscriber
                self.nextstate = -1

        # executes these commands only if a -2 command (exit) has been given.
        # It moves to the previous state and updates the output variables.
        self.statename = 'JOG STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'START JOG MODE STATE'
        return 'tJog'


class JogStepSize(State):
    """ State to change the Jog Step size. Can be accessed from the base Jog State
    or from the Run Jog State. It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tJog', 'tJRun'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        # no need to initialize, it simply opens a file
        self.statename = 'CHANGE JOG STEP SIZE'
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 4: ' + self.statename + ']' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '-- OPENING POINTS FILE --' + color.END)
        # calls gedit "filepath" in the shell, so to edit the step size manually
        return_code = subprocess.call("gedit /home/ros/catkin_ws/src/state_machine_package/data/JogStepSize.txt", shell=True)
        rospy.loginfo(color.BOLD + color.PURPLE + '-- DONE!--' + color.END)
        # reloads the file after the editing process ended
        step = load_txt('/home/ros/catkin_ws/src/state_machine_package/data/JogStepSize.txt')
        # prints the new step size as a debug info
        rospy.loginfo(color.BOLD + color.YELLOW + '| NEW STEP SIZE: ' + str(step[0][0]) + ' |' + color.END)
        userdata.jogstep = float(step)
        # goes back to previous state, depending on which one was it

        if userdata.oldstate == 'JOG MODE STATE':
            # if the previous state was the base Jog state, returns there
            self.statename = 'JOG STATE'
            rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
            userdata.oldstate = 'CHANGE JOG STEP SIZE STATE'
            # no need to change the request number since it has not been used here
            userdata.output = userdata.input
            return 'tJog'
        else:
            # returns to the Run Jog State, since it's the only other option
            self.statename = 'MOVE JOG STATE'
            rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
            userdata.oldstate = 'CHANGE JOG STEP SIZE STATE'
            # no need to change the request number since it has not been used here
            userdata.output = userdata.input
            return 'tJRun'

########### SET SYSTEM VELOCITY ###########

''' CONTROLLA I VALORI DELLA VELOCITA' COME VENGONO SETTATI (0.9 o 90 ecc) '''
class SetVel(State):
    """ State to set the joint velocity at system level. By doing this every movement packet
    will also set the correct value accordingly. It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tHome'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|  COMMAND LIST  |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 1: 10% SPEED   |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 2: 20% SPEED   |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: 100% SPEED  |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: RETURN HOME |' + color.END)

    def execute(self, userdata):
        # initializes variables
        self.statename = 'SET ROBOT MOVEMENT SPEED'
        self.update()
        # updates the request number
        self.request.request_number = userdata.input + 1
        # prints a debug info and the command list
        rospy.loginfo(color.BOLD + color.CYAN + '[STATE 5: ' + self.statename + ']' + color.END)
        self.listCommands()
        # sets the request type to change state since nextstate its the variable we want to update
        self.request.request_type = 'change_state'
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # reads the value of nextstate using the correct callback function and request type
            self.sub = rospy.Subscriber('/command_response', commandResponse, self.stateCallback, queue_size = 1)
            # note that is nextstate that gets updated, but a numerical command is given representing the speed value
            # in a similar way to what is done in the home state to select the substates.
            # here, selecting numbers from 1 to 9 select a velocity from 10% to 90%, while 0 means 100%
            if (self.nextstate >= 0) and (self.nextstate <= 9):
                # if a number from 0 to 9 has been read
                if self.nextstate == 0:
                    # if 0 then sets the maximum velocity
                    # TO DO: how to set velocity? what is the value?
                    vel = 100
                else:
                    # else is a number from 1 to 9, so to get the percentage we need to multiply it by 10
                    vel = self.nextstate * 10

                self.statename = 'SPEED ' + str(vel) + '%'
                self.confirm()
                # asks the user to confirm the value
                if self.command == 0:
                    # after confirming, it sets the variable globally. Every movement packet after this modification
                    # will be sent with the velocity parameter set accordingly.
                    rospy.loginfo(color.BOLD + color.GREEN + 'CHANGING TO ROBOT SPEED: ' + str(vel) + '%' + color.END)
                    # sets the new velocity parameter
                    userdata.velocity = vel/100
                    # after this parameter has been set, it exits from the state and returns home
                    self.nextstate = -2
                    break
                else:
                    # stays inside the while loop waiting
                    self.nextstate = -1

            elif self.nextstate == -2:
                # if the exit command is given, it exits from the state
                break

            else:
                # waits until something useful is read from the topic
                self.nextstate = -1

        # these commands are conly executed if nextstate is -2 after breaking the while.
        # They allow to go back to previous state, home in this case
        self.statename = 'HOME STATE'
        rospy.loginfo(color.BOLD + color.RED + '-- RETURNING TO ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_type
        userdata.oldstate = 'SET ROBOT MOVEMENT SPEED STATE'
        return 'tHome'

########### MAIN DEFINITION AND CALL ###########

def myhook():
    # just prints an info log at shutdown
    rospy.loginfo(color.BOLD + color.RED + '\n -- KEYBOARD INTERRUPT, SHUTTING DOWN --' + color.END)

def main():
    # init node
    rospy.init_node('smach_state_machine')
    # until it is not shutdown
    while not rospy.is_shutdown():
        # Create a SMACH state machine
        # the only outcome possible right now is that the state machine exits!
        sm = smach.StateMachine(outcomes=['exit'])
        # here I could also add "input_keys=['sm_input'], output_keys=['sm_output'])"

        # userdata specifies inputs and outputs to be remapped afterwards.
        # the initial request number is zero and the initial preceding state is none
        sm.userdata.request_number = 0
        sm.userdata.oldstate = 'NONE'
        # sets the robot velocity to 80% system wise
        sm.userdata.velocity = 0.80
        # sets the jog step size system wise loading it from file
        sm.userdata.jogstep = load_txt('/home/ros/catkin_ws/src/state_machine_package/data/JogStepSize.txt')
        sm.userdata.jogstep = sm.userdata.jogstep[0][0]

        # Open the container
        with sm:
            # Add states to the container.
            # Remember to set the state name + calling the state(), setting its transitions correctly and performing
            # a remapping of inputs and outputs!!
            smach.StateMachine.add('ReadyState', ReadyState(),
                                    transitions={'tHome':'Home', 'texit':'exit'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('Home', Home(),
                                    transitions={'tState0':'ReadyState',
                                    'tPointsDef':'PointsDef', 'tLoop':'Loop',
                                    'tPickPlace':'PickPlace', 'tJog':'Jog', 'tSetVel':'SetVel'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('PointsDef', PointsDef(),
                                    transitions={'tHome':'Home'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('Loop', Loop(),
                                    transitions={'tHome':'Home',
                                    'tLTraj':'LoopTrajectory',
                                    'tLRun':'LoopRun'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('LoopTrajectory', LoopTrajectory(),
                                    transitions={'tLoop':'Loop'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('LoopRun', LoopRun(),
                                    transitions={'tLoop':'Loop'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('PickPlace', PickPlace(),
                                    transitions={'tHome':'Home'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('Jog', Jog(),
                                    transitions={'tHome':'Home',
                                    'tJRun':'JogRun',
                                    'tJStep':'JogStepSize'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('JogRun', JogRun(),
                                    transitions={'tJog':'Jog', 'tJStep':'JogStepSize'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('JogStepSize', JogStepSize(),
                                    transitions={'tJog':'Jog', 'tJRun':'JogRun'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('SetVel', SetVel(),
                                    transitions={'tHome':'Home'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('state_machine_server', sm, '/ROOT')
        sis.start()

        # Execute SMACH plan
        outcome = sm.execute()

        # stops and quits
        sis.stop()
        rospy.on_shutdown(myhook)
        sys.exit(0)

if __name__ == '__main__':
    main()
