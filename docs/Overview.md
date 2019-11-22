# Overview of the system
The system is composed of several nodes that perform some core functionality. The nodes talk with each other by sending messages on specific topics.
At the present time, the developed structure is detailed in Fig. 1.

<p align="center">
  <img height="400" src="https://github.com/Krissy93/meta-workstations-project/blob/master/images/system_overview3.png">
</p>

## State Machine Node
The main node is the **State Machine Node**, which contains the definition of the basic state machine used to command the robot.
For now, a set of simple functionalities (Fig. 2) has been developed but new robot functionalities can be easily added to the state machine by adding a new State to the `state_machine.py` core file.

<p align="center">
  <img src="https://github.com/Krissy93/meta-workstations-project/blob/master/images/statemachine2.png">
</p>

The functionalities developed so far are:
- **Ready State:** This is the first state of the State Machine. From there, the user can only QUIT the program or proceed to the HOME state (CONFIRM gesture). It is redundant on purpose to avoid the closing of the application by mistake;
- **Home State:** In this state the user can select the functionality to use simply by performing the numerical gesture related to the state (i. e. 0 + 1 is the Loop State).
- **Points Definition State:** It is accessed by performing the "OPEN FILE" gesture. The only purpose of this state is to open the `SPoints` or `QPoints` files, letting the user modify them by using the keyboard and update them upon recalculating their corresponding transforms (i. e. fromt SPoints one can obtain the corresponding QPoints by using the robot Kinematic functions);
- **Loop State:** It is accessed by performing the 0 + 1 numerical gesture. Users can modify the Operations file accessing the Loop Operation State ("OPEN FILE") or launch a Loop Operation ("CONFIRM" gesture). In this case, users have to select the Operation from the list and choose the number of times this loop must be executed. Loops can be stopped or paused any time;
- **Pick & Place State:** This state is accessed by performing the 0 + 2 numerical gesture. Users interactively choose an action from the list of actions and, if required by the action, a point from the list of points. Upon selection, the command is sent to the robot and after its execution the system asks the user if it wants to select another action to be performed or if it wants to end the procedure, in this case going back to the Home State;
- **Jog State:** It is the state where users can change the robot joint positions easily. It is accessed by performing the 0 + 3 numerical gesture, and it has two sub-states: the first one ("OPEN FILE" gesture) allows users to change the robot step size by modifiying the related file, while the second one ("CONFIRM" gesture) allows users to launch the Jog Mode. In Jog Mode each joint is moved one at a time of the Jog Step Size, and users can decide to: move the joint increasing its position (Joint + Right Direction gesture), move the joint decreasing its position (Joint + Left Direction gesture) or stop the joint right there (Joint + Five gesture). To save the achieved position, users can perform the "CONFIRM" gesture in order to append its joint coordinates read from the `/joint_states` topic to the `QPoints` file;
- **Set System Velocity State:** This state is accessed by performing the 0 + 4 gesture. Users can define the robot velocity system-wise by performing the numerical gesture related to the percentage of speed they want. For example, to set the robot velocity at 80% users must perform the 0 + 8 gesture.

## Gesture Recognition Node
To use the State Machine the user must give it commands in two ways:
- By using the **Keyboard Node:** in this case the commands must be numbers according to the available command list of each state (printed every time the user enters a state).
- By using the **Gestures Node:** in this case the user performs some hand gestures following a specific code. The gestures are then translated into numerical commands according to what the camera sees.

[Here](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Gestures%20Node.md) the node idea is explained in detail. The functionality of the node is the same even if the Keyboard version is used, just remember to write the numerical command corresponding to the gesture you want to perform.

## Robot Driver Node
To send the operative instruction to the robot and make it move, the State Machine sends points coordinates to the **Robot Driver** using the [JointTrajectory message](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html), which is published in the `/joint_path_command` topic.
Keep in mind that if you want the robot to execute a complex trajectory composed of multiple robot positions, it is best to send each position individually to be able to stop or pause the robot between each message.
Otherwise, the robot receives the whole list of positions (defines as a [JointTrajectoryPoints message](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)) and executes them in sequence.
It cannot be stopped or paused from the State Machine, but only from traditional command methods (Teach pad, Robot proprietary software etc...).

Robot Drivers are robot-dependant. A list of available drivers can be found [here](http://wiki.ros.org/Industrial/supported_hardware).
If the driver for the specific robot you have is not in the list, you have to write your own!

To receive a feedback from the robot, the [JointState message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) is used.
Robot Drivers usually publish these messages with a high rate in the corresponding feedback topic (`/joint_states` in our case), so to understand if the robot has reached the final position (or theoretical position, the one sent as a request) the callback function must continuosly check the received message.
