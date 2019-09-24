#!/usr/bin/env python

import rospy
import rospkg
import smach
import smach_ros
import sys
import time
import subprocess
import glob
from state_machine_package.msg import commandRequest, commandResponse
import threading
import traceback
from utils import *
from actions_library_sawyer import *
# sawyer interface
import intera_interface

########### DEFINITION OF MACRO ###########

PKG_PATH = rospkg.RosPack().get_path('state_machine_package')
sys.path.insert(1, PKG_PATH + '/operations/')
REQUEST_TOPIC = '/command_request'
RESPONSE_TOPIC = '/command_response'
ROBOT_PUB = '/joint_state_pose'
ROBOT_FEEDBACK = '/joint_states'
start_execution = time.time()

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
        self.pub = rospy.Publisher(REQUEST_TOPIC, commandRequest, queue_size = 1)
        self.trajpub = rospy.Publisher(ROBOT_PUB, JointTrajectory, queue_size = 1, latch=True)
        self.request = commandRequest()
        self.trajectory = JointTrajectory()
        # basic empty vector useful for initialization of velocities accelerations and effort
        # it is a vector of 10 elemets, which is the maximum. It is okay because messages
        # are created based on the position vector, which is robot dependant
        self.empty = [0,0,0,0,0,0]

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
        if (msg.position >= self.trajectory.positions - 0.01) and (msg.position <= self.trajectory.positions + 0.01):
        #print(self.trajectory.points[0].positions)
        #if (msg.position[0] == 888) and (msg.position[1] >= self.trajectory.points[0].positions[1] - 0.01) and (msg.position[1] <= self.trajectory.points[0].positions[1] + 0.01):
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
        self.request.active_state = self.statename
        self.request.header.stamp = rospy.Time.now()
        self.pub.publish(self.request)

    def confirm(self):
        """ Function that checks if the user confirms, cancels or ask to go back home.
        It's called after a transition command has been given to the state machine,
        so it's common between the states and it's inherited from the parent class State. """

        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ': CONFERMI?]' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|         COMMAND LIST         |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: CONFERMA COMANDO INSERITO |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-3: ANNULLA E REINSERISCI     |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: ANNULLA ED ESCI           |' + color.END)
        # updates request number and type, since it waits for a confirmation command to confirm the transition
        self.update('interface_command')

        while self.command == -1:
            # reads the response topic and waits for the correct response to appear
            self.sub = rospy.Subscriber(RESPONSE_TOPIC, commandResponse, self.stateCallback, queue_size = 1)

            if self.command == -3:
                # cancels the inserted command. After this command, the state machine exits the "confirm"
                # method but still waits inside the changestate method.
                rospy.loginfo(color.BOLD + color.YELLOW + '-- CANCELLATO, REINSERIRE --' + color.END)
                break
            elif self.command == 0:
                # confirms the inserted command. Exits the "confirm" method and in the
                # changestate method performs the corresponding action.
                rospy.loginfo(color.BOLD + color.GREEN + '-- CONFERMATO --' + color.END)
                break
            elif self.command == -2:
                # exits the state, returning to the preceding one.
                rospy.loginfo(color.BOLD + color.RED + '-- ANNULLATO, USCITA --' + color.END)
                break
            else:
                # waits
                self.command = -1

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
        self.statename = 'STATO INIZIALE'
        self.reset()
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input + 1

        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ': PRONTO A COMUNICARE]' + color.END)
        self.listCommands()
        # from this state the only action possible is to change the state or close communication,
        # thus the request type is initialized as "change state"
        self.request.request_type = 'change_state'
        self.request.active_state = self.statename
        self.request.header.stamp = rospy.Time.now()
        # sleep is needed to correctly send the request in the topic,
        # since at startup it may not be already up if the user is fast and already sends a command
        time.sleep(1)
        #print(self.request)
        self.pub.publish(self.request)

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber(RESPONSE_TOPIC, commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == 0:
                self.statename = 'STATO HOME'
                self.confirm()
                # asks the user to confirm the state read from the response message.

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'STATO INIZIALE'
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
        rospy.loginfo(color.BOLD + color.RED + '-- CHIUSURA DEL PROGRAMMA --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'STATO INIZIALE'
        return 'texit'

class Home(State):
    """ Home state. From here, the user can choose what to do, transitioning to the
    different states. New functionalities must be connected to this state to be accessed!
    It inherits common traits from the parent State class. """

    def __init__(self, transitions=['tState0','tHello', 'tUserMove', 'tUtenti', 'tCerchio'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)
        self.statename = 'STATO HOME'

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|         COMMAND LIST         |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 1: TRAIETTORIA "HELLO"       |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 2: MUOVI TU IL SAWYER!       |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 3: CARICA TRAIETTORIA UTENTE |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 4: TRAIETTORIA "CERCHIO"     |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: TORNA A STATO INIZIALE    |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'STATO HOME'
        # needed to correctly set the request number when entering the state
        self.request.request_number = userdata.input
        # prints list of commands and debug info
        rospy.loginfo(color.BOLD + color.CYAN + '[' + self.statename + ': COSA VUOI FARE?]' + color.END)
        self.listCommands()
        # from this state the only action available is to change state or exit,
        # thus the command type is a change state one.
        self.update('change_state')

        while self.nextstate == -1:
            # waits for the correct command according to the request number sent before
            self.sub = rospy.Subscriber(RESPONSE_TOPIC, commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == 1:
                # chooses the point definition state
                self.statename = 'HELLO'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'STATO HOME'
                    return 'tHello'
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
                self.statename = 'STATO MOVIMENTO UTENTE'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tUserMove'
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
                # chooses the pick and place state
                self.statename = 'STATO SCELTA TRAIETTORIA'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tUtenti'
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
                # chooses the loop state
                self.statename = 'CERCHIO'
                self.confirm()
                # asks the user to confirm the selection

                if self.command == 0:
                    # if the command has been confirmed, updates the output to be sent
                    # to the following state as the current request number, and keeps track
                    # of the state from when it comes from in "oldstate". After this, it performs the transition
                    userdata.output = self.request.request_number
                    userdata.oldstate = 'HOME STATE'
                    return 'tCerchio'
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
        self.statename = 'STATO INIZIALE'
        rospy.loginfo(color.BOLD + color.RED + '-- CHIUDO COMUNICAZIONE, RITORNO A ' + self.statename + ' --' + color.END)
        userdata.output = self.request.request_number
        userdata.oldstate = 'STATO HOME'
        return 'tState0'

########### HELLO STATE DEFINITION ###########

class Hello(State):

    def __init__(self, transitions=['tHome'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        self.statename = 'HELLO'
        rospy.loginfo(color.BOLD + color.CYAN + '-- BENVENUTO NELLO STATO ' + self.statename + '! --' + color.END)

        # esegue operazioni previste dallo stato hello
        hello()

        # returns home automatically after these operations are concluded.
        self.statename = 'STATO HOME'
        rospy.loginfo(color.BOLD + color.RED + '-- RITORNO A ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'HELLO'
        return 'tHome'

class Cerchio(State):

    def __init__(self, transitions=['tHome'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        self.statename = 'TRAIETTORIA CIRCOLARE'
        rospy.loginfo(color.BOLD + color.CYAN + '-- BENVENUTO NELLO STATO ' + self.statename + '! --' + color.END)

        # esegue operazioni previste dallo stato hello
        cerchio()

        # returns home automatically after these operations are concluded.
        self.statename = 'STATO HOME'
        rospy.loginfo(color.BOLD + color.RED + '-- RITORNO A ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'HELLO'
        return 'tHome'

########### LOOP STATES ###########

class UserMove(State):
    ''' Loop state. The operator selects the operation to be executed from a list of
    available operations, loads it into memory then says to the robot how many times it
    must execute the loop. It can be paused or stopped in the time between the sending of packets.
    It inherits common InstructionState methods. '''

    def __init__(self, transitions=['tHome', 'tUtenti'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def listCommands(self):
        """ Method called at the startup of the state, to present the user the command list available. """

        rospy.loginfo(color.BOLD + color.PURPLE + '|          COMMAND LIST         |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '| 0: CONFERMA INSERIMENTO PUNTO |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-5: TERMINA INSERIMENTO PUNTI  |' + color.END)
        rospy.loginfo(color.BOLD + color.PURPLE + '|-2: TORNA ALLA HOME            |' + color.END)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'STATO MOVIMENTO UTENTE'
        # the request number do not need to be updated here
        self.request.request_number = userdata.input

        # asks the user if it wants to add another operation to the loop or just send the loop as it is
        self.listCommands()
        # updates the variables and publishes a new request
        self.update('change_state')

        # crea il nome del filepath
        BASENAME = PKG_PATH + '/traiettorie/Utente_'
        date = time.strftime(r"%H:%M:%S", time.localtime())
        filename = BASENAME + date + '.txt'

        # printa all'utente il nome del file su cui salvera' le cose
        rospy.loginfo(color.BOLD + color.CYAN + '-- BENVENUTO! --' + color.END)
        rospy.loginfo(color.BOLD + color.YELLOW + '-- NOTA: SALVO I TUOI DATI NEL FILE CHIAMATO: Utente_' + str(date) + '.txt --' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + '-- MUOVIMI DOVE VUOI, POI FAI IL GESTO DI CONFERMA PER SALVARE IL PUNTO! --' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + '-- PER TERMINARE DI INSERIRE PUNTI E LANCIARE IL PROGRAMMA, FAI IL GESTO DI TERMINE! --' + color.END)

        while self.nextstate == -1:
            self.sub = rospy.Subscriber(RESPONSE_TOPIC, commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate == 0:
                # chiede la conferma
                self.statename = 'INSERISCI PUNTO'
                self.confirm()
                # check interface
                if self.command == 0:
                    # user confirmed the value
                    punto = limb.joint_angles()
                    point2file(filename, punto)
                    rospy.loginfo(color.BOLD + color.GREEN + '-- PUNTO SALVATO! --' + color.END)
                    self.listCommands()
                    self.update('change_state')
                elif self.command == -2:
                    # exits state
                    self.nextstate = -2
                    break
                else:
                    # inserted delete, need to update
                    self.listCommands()
                    self.update('change_state')


            elif self.nextstate == -5:
                # terminato l'inserimento eseguo la traiettoria
                self.statename = 'TERMINA INSERIMENTO E LANCIA PROGRAMMA'
                self.confirm()

                if self.command == 0:
                    # user confirmed the value
                    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER INIZIA A MUOVERSI, ATTENZIONE! --' + color.END)
                    go2trajectory(filename)
                    # quando finisce esce e torna in home
                    self.nextstate = -2
                    break
                elif self.command == -2:
                    # exits state
                    self.nextstate = -2
                    break
                else:
                    # inserted delete, need to update
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate != 0 and self.nextstate != -5 and self.nextstate =! -1:
                # se non e' nessuno di questi casi, allora e' un gesto sbagliato che non corrisponde a nulla e devo mandare una richiesta nuova
                rospy.loginfo(color.BOLD + color.RED + '-- GESTO SCONOSCIUTO, RIPROVARE --' + color.END)
                self.listCommands()
                self.update('change_state')

            else:
                # just waits for the subscriber to read something useful
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Returns to the home state and updates variables (useful for debug)
        # returns home automatically after these operations are concluded.
        self.statename = 'STATO HOME'
        rospy.loginfo(color.BOLD + color.RED + '-- RITORNO A ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'STATO MOVIMENTO UTENTE'
        return 'tHome'

class UserChoice(State):
    ''' Loop state. The operator selects the operation to be executed from a list of
    available operations, loads it into memory then says to the robot how many times it
    must execute the loop. It can be paused or stopped in the time between the sending of packets.
    It inherits common InstructionState methods. '''

    def __init__(self, transitions=['tHome', 'tUserMove'], input=['input', 'oldstate', 'velocity', 'jogstep'], output=['output', 'oldstate', 'velocity', 'jogstep']):
        State.__init__(self, transitions, input, output)

    def execute(self, userdata):
        # when executed, initializes the variables calling the reset method
        self.statename = 'STATO SCELTA TRAIETTORIA'
        # the request number do not need to be updated here
        self.request.request_number = userdata.input
        # updates the variables and publishes a new request
        self.update('change_state')

        # printa all'utente il nome del file su cui salvera' le cose
        rospy.loginfo(color.BOLD + color.CYAN + '-- BENVENUTO! --' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + '-- CHE FILE DI TRAIETTORIA VUOI CARICARE? --' + color.END)
        rospy.loginfo(color.BOLD + color.CYAN + '-- ESEGUI IL GESTO CORRISPONDENTE AL NUMERO DEL FILE PER CARICARLO! --' + color.END)

        saved_files = get_files(PKG_PATH)

        for i in range(0,len(saved_files)):
            rospy.loginfo(color.BOLD + color.YELLOW + '| FILE ' + str(i+1) + ': ' + str(os.path.basename(saved_files[i])) + ' |' + color.END)

        while self.nextstate == -1:
            self.sub = rospy.Subscriber(RESPONSE_TOPIC, commandResponse, self.stateCallback, queue_size = 1)

            if self.nextstate > 0:
                # se numero
                # chiede la conferma
                value = self.nextstate
                self.statename = 'SELEZIONATO FILE NUMERO' + str(value)
                self.confirm()
                # check interface
                if self.command == 0:
                    # user confirmed the value
                    # carica file e lancia traiettoria
                    filename = saved_files[value]
                    rospy.loginfo(color.BOLD + color.GREEN + '-- FILE ' + str(os.path.basename(saved_files[value])) + 'CARICATO! --' + color.END)
                    rospy.loginfo(color.BOLD + color.YELLOW + '-- SAWYER INIZIA A MUOVERSI, ATTENZIONE! --' + color.END)
                    go2trajectory(filename)
                    self.nextstate = -2
                    break

                elif self.command == -2:
                    # exits state
                    self.nextstate = -2
                    break
                else:
                    # inserted delete, need to update
                    self.listCommands()
                    self.update('change_state')

            elif self.nextstate < -1 and self.nextstate > 100 and self.nextstate =! -1:
                # se non e' nessuno di questi casi, allora e' un gesto sbagliato che non corrisponde a nulla e devo mandare una richiesta nuova
                rospy.loginfo(color.BOLD + color.RED + '-- GESTO SCONOSCIUTO, RIPROVARE --' + color.END)
                self.listCommands()
                self.update('change_state')

            else:
                # just waits for the subscriber to read something useful
                self.nextstate = -1

        # these commands are only executed if nextstate = -2, since it exits the while loop only in this case.
        # Returns to the home state and updates variables (useful for debug)
        # returns home automatically after these operations are concluded.
        self.statename = 'STATO HOME'
        rospy.loginfo(color.BOLD + color.RED + '-- RITORNO A ' + self.statename + ' --' + color.END)
        # since no request has been sent, the request number is the same thus is passed as it is.
        userdata.output = userdata.input
        userdata.oldstate = 'STATO SCELTA TRAIETTORIA'
        return 'tHome'

########### MAIN DEFINITION AND CALL ###########

def myhook():
    # just prints an info log at shutdown
    end_execution = time.time()
    rospy.loginfo(color.BOLD + color.RED + '\n -- KEYBOARD INTERRUPT, SHUTTING DOWN. ELAPSED TIME: ' + str(end_execution - start_execution) + ' SECONDS --' + color.END)

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
        sm.userdata.jogstep = load_txt(PKG_PATH + '/data/JogStepSize.txt')
        sm.userdata.jogstep = sm.userdata.jogstep[0][0]
        sm.userdata.operations = []

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
                                    'tHello':'Hello', 'tUserMove':'UserMove',
                                    'tUtenti':'UserChoice', 'tCerchio':'Cerchio'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('Hello', Hello(),
                                    transitions={'tHome':'Home'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('UserMove', UserMove(),
                                    transitions={'tHome':'Home',
                                    'tUtenti':'UserChoice'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep'})
            smach.StateMachine.add('UserChoice', UserChoice(),
                                    transitions={'tHome':'Home', 'tUserMove':'UserMove'},
                                    remapping={'input':'request_number',
                                    'output':'request_number',
                                    'oldstate':'oldstate',
                                    'velocity':'velocity',
                                    'jogstep':'jogstep',
                                    'operations':'operations'})
            smach.StateMachine.add('Cerchio', Cerchio(),
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
        if outcome == 'exit':
            # stops and quits
            sis.stop()
            rospy.on_shutdown(myhook)
            sys.exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
