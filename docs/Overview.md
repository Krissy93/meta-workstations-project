# Overview of the system
The system is composed of several nodes that perform some core functionality. The nodes talk with each other by sending messages on specific topics.
At the present time, the developed structure is detailed in Fig. 1.

<p align="center">
  <img height="400" src="https://github.com/Krissy93/meta-workstations-project/blob/master/images/system_overview.png">
</p>

## State Machine Node
The main node is the **State Machine Node**, which contains the definition of the basic state machine used to command the robot.
For now, a set of simple functionalities (Fig. 2) has been developed but new robot functionalities can be easily added to the state machine by adding a new State to the `state_machine.py` core file.

<p align="center">
  <img src="https://github.com/Krissy93/meta-workstations-project/blob/master/images/Fig2.png">
</p>

Detailed information on the functionalities can be found in the corresponding file of the list:
- Home State
- Points Definition State
- Loop State
- Pick & Place State
- Jog State
- Set System Velocity State

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
