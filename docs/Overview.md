# Overview of the system
MEGURU is composed of several nodes that perform core functionalities. The nodes talk with each other by sending messages on specific topics.
A detailed overview of the system can be found in [our paper](https://doi.org/10.1016/j.rcim.2020.102085): check it out to learn more about it and about the experimental validation carried out!

## State Machine Node
The main node is the **State Machine Node**, which contains the definition of the basic state machine used to command the robot.
For now, a set of simple functionalities has been developed but new robot functionalities can be easily added to the state machine by adding a new State to the `state_machine.py` core file.

When launched, the State Machine is in the initial state, called **Ready State**. In this state, the State Machine waits for the user Commands and can move to (i) the **EXIT** State (where the program quits) or (ii) to the **Home State**. From this state, the user can access to one out of the following four states: 
1. **SOPs Building State (SB):** this state is accessed by performing 0 + 1 gestures. This is the core state of the State Machine. Here, the user selects Actions from the Action library and, if requested by the Action, selects Points from the Point File. Each single Action results in the corresponding robot task and is immediately executed by the robot. Complex tasks are implemented as SOPs built by means of a user-machine collaboration that combines different Actions and saves them in the corresponding Operation file;
2. **COPs Building State (CB):** this state is accessed by performing 0 + 2 gestures. In this state COPs are built. To this aim, the State Machine enters in a loop where the user can select, for each single SOP, the corresponding Operation file and the number of iterations that SOP must be repeated; then, the State Machine moves to the COPs Launch State (CL) where each single Action of the COP is sent to the robot until the whole sequence is performed. To do so, the user must select one or more Operation files that are loaded automatically at the start of this state from the corresponding subdirectory of the package (e. g. `/state_machine_package/operations`). When an Operation file is selected, the user tells the robot how many repetitions of the last Operation file it must execute before proceeding to the next Operation file.
    For example:
    ```
    Operation 1: /operations/First.py
    Repetitions: 3
    Operation 2: /operations/Second.py
    Repetitions: 2
    ```
    The final schedule of operations for the robot are:
    ```
    Operation 1
    Operation 1
    Operation 1
    Operation 2
    Operation 2
    ```
    It is also possible to define the whole Operation as a number of Operation files and define a loop value for the whole Operation:
    ```
    Operation 1: /operations/First.py
    Repetitions: 1
    Operation 2: /operations/Second.py
    Repetitions: 1
    Global Repetitions: 4
    ```
    In this case, the schedule is:
    ```
    Operation 1
    Operation 2

    Operation 1
    Operation 2

    Operation 1
    Operation 2

    Operation 1
    Operation 2
    ```
    The execution of each Action can be paused by the user using the **PAUSE** gesture and resumed by performing the **CONFIRM** gesture. It is also possible to stop the execution completely by performing the **EXIT** gesture at any time: in this case the State Machine moves back to the Ready State;
3. **Jog State (J):** this state is accessed by performing 0 + 3 gestures. This is a service state designed to help users to perform maintenance checks on the robot motors and to set specific positions of the robot according to their needs. It is divided in two states: the former is the **Jog Mode State (JM)** (accessed by performing the **CONFIRM** gesture), where operators can increment or decrement the joint position of the robot according to the joint selected using the corresponding gesture, or stop the joint in the current position (**Joint + Right Direction/Left Direction/Five gesture** respectively, users can also save a certain position by performing the **CONFIRM** command without exiting or pausing the state: this will append the new joint coordinates read from the `/joint_states` topic to the `QPoints` file); the latter is the **Jog Step State (JS)** (accessed by performing the **OPEN FILE** gesture), where operators can increase or decrease the default Jog step size by changing the parameter in the corresponding file;
4. **Robot Speed State (RS):** This state is accessed by performing the 0 + 4 gesture. In this state, the movement speed of the robot can be modified. The default speed is set to 100%, but users can decrease it by performing the gesture which correspond to a lower percentage of the total speed of the robot (e. g. to obtain the 80% of the total speed, the user must perform the instruction which corresponds to number 8, thus 0 + 8). It is worth noting that the specified speed is automatically set upon sending a new robot movement message, without interfering with the proprietary controller settings, by setting the corresponding parameter of the message.

## Gesture Recognition Node
To use the State Machine the user must give it commands in two ways:
- By using the **Keyboard Node:** in this case the commands must be numbers according to the available command list of each state (printed every time the user enters a state).
- By using the **Gestures Node:** in this case the user performs some hand gestures following a specific code. The gestures are then translated into numerical commands according to what the camera sees.

[Here](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Gestures%20Node.md) the node idea is explained in detail. The functionality of the node is the same even if the Keyboard version is used, just remember to write the numerical command corresponding to the gesture you want to perform.

## How to move the robot?
To send the operative instruction to the robot and make it move, the State Machine sends points coordinates to the robot using the [JointTrajectory message](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html), which is published in the `/joint_path_command` topic.
Keep in mind that if you want the robot to execute a complex trajectory composed of multiple robot positions, it is best to send each position individually to be able to stop or pause the robot between each message.
Otherwise, the robot receives the whole list of positions (defines as a [JointTrajectoryPoints message](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)) and executes them in sequence.
It cannot be stopped or paused from the State Machine, but only from traditional command methods (Teach pendant, Robot proprietary software etc...).

Keep in mind that **robot drivers** are robot-dependant, you need them to translate ROS commands to the robot if a specific ROS wrapper/interface doesn't exist. A list of available drivers can be found [here](http://wiki.ros.org/Industrial/supported_hardware).
If the driver for the specific robot you have is not in the list, you have to write your own!

To receive a feedback from the robot, the [JointState message](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) is used.
Robot Drivers usually publish these messages with a high rate in the corresponding feedback topic (`/joint_states` in our case), so to understand if the robot has reached the final position (or theoretical position, the one sent as a request) the callback function must continuosly check the received message.
