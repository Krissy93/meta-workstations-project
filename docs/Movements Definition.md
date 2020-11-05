# Movements Definition
At the present time, the software is not programmed to automatically understand high-level commands such as "Assemble object" or "Take object".
Every instruction given to the robot must correspond to an operation already written down somewhere in an "hard-code" way.
This basically is what users usually do when programming a robot for a specific task: they write down in its code what it has to do according to the set-up where it is mounted on.

Because of this, we defined three main types of hard-code building blocks definitions:
- **POINTS:** these are the base blocks that compose MEGURU programming structure. They represent the positions of the robot end effector in the current reference system, expressed by (i) cartesian coordinates or by (ii) Joints position. Points are collected by the user either using ROS or the robot proprietary software, and are stored in an ordered list, thereafter called “Points file”;
- **ACTIONS:** these are parametric functions that represent a simple action of the robot (e. g. opening and closing a gripper or moving the robot to a certain point). A dedicated Python library based on ROS communication functionalities has been developed to (i) guarantee the independency of the Actions from the robot manufacturer platform and (ii) allow the user to easily define different Actions according to the application needs;
- **OPERATIONS:** by using MEGURU, users can build two types of Operations: **Simple Operations (SOPs)**, obtained by combining different Actions, and **Combined Operations (COPs)**, obtained by joining multiple SOPs. As a result, MEGURU allows users to reconfigure the robot tasks in reduced times and to adapt the robot to a mixed production minimizing production downtime.

## How to define Points
Users move the robot in the space by using their preferred method (e. g. manual guidance or ROS interfaces or Teach Pendant, etc.), then save the current position in both Cartesian coordinates and Joint states. Right now, the software does not this automatically, thus points must be written manually in certain .txt files.

The corresponding files are [SPoints](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/data/SPoints.txt) and [QPoints](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/data/QPoints.txt).
SPoints are saved in their XYZ-Orientation cartesian positions. These can be read and elaborated in the Points Definition State, in order to transform them in Joint State positions according to the robot at hand. To do so, one must modify the [Robot.py](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/src/robot.py) file with the correct Robot structure and operative angles for each joint, in order to obtain the correct Direct and Inverse Kinematic functions.
One can also add a QPoint simply by reading the `/joint_state` position of the robot and add it to the list.

SPoints are defined following this structure:
```
POINT_NAME  X Y Z ANGLE
```
While QPoints are defined following this structure:
```
POINT_NAME  JOINT1 JOINT2 ... JOINTN
```
Be careful: the only separator between values is the space!!

## How to define Actions
Actions are defined in the [actions_library](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/src/actions_library.py) file. It's a python file imported in the State Machine Node, and contains several action functions.
Actions are defined in a parametric way, allowing the user to call a generic action with the parameters needed at that time.
To add a new action, simply add the corresponding function to the python file.

In this case the action name is the name of the function called.
The structure of an action is similar to this:
```
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
    # then other actions can be performed afterwards
```
Right now, only 3 actions have been defined:
- Move to point
- Open gripper (which calls a service connected to the robot gripper)
- Close gripper (same as before)

## How to define Operations
Operations are complete python programs that can be loaded into memory and executed as they are or joined together with other Operations to compose a more complex one. 

Each Operation file has an object defined at the start of the program that is loaded into memory. The object contains the type of Actions (e. g. Move to point is Action 1) and their parameters, defining the Operation as an ordinate sequence of Actions to take. It is easy to join more Operations together in this way, or tell the interface to reproduce the Operation sequence a given number of times before starting the new Operation sequence.

Operations are used in the **COPs Building State**, and can be defined by hand by the user or from the **SOPs Building State**. In this case, the user performs the Actions one at a time and, upon exiting the State, can save the Actions taken with their parameters into an Operation file.
