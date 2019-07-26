# Movements Definition
At the present time, the software is not programmed to automatically understand high-level commands such as "Assemble object" or "Take object".
Every instruction given to the robot must correspond to an operation already written down somewhere in an "hard-code" way.
This basically is what users usually do when programming a robot for a specific task: they write down in its code what it has to do according to the set-up where it is mounted on.

Because of this, we defined three main types of hard-code movement definitions:
- **POINT DEFINITION:** where the user simply adds a point coordinate to a list of points;
- **ACTION DEFINITION:** where the user defines a set of operations that compose a more complex instruction, i. e. to take a specific object one must specify where to move the end effector, how to open the gripper, how to take the object with the gripper, etc.
Simple actions are useful to compose more complex workflows, but are robot and set-up dependant;
- **OPERATION DEFINITION:** operations are a complete workflow composed of a set of simple actions. These are defined by calling the specific simple action in the correct time of the workflow.

## Point Definition
The corresponding files are [SPoints](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/data/SPoints.txt) and [QPoints](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/data/QPoints.txt).
SPoints are saved in their XYZ-Orientation cartesian positions. These can be read and elaborated in the Points Definition State, in order to transform them in Joint State positions according to the robot at hand. To do so, one must modify the [Robot.py](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/src/robot.py) file with the correct Robot structure and operative angles for each joint, in order to obtain the correct Direct and Inverse Kinematic functions.
One can also simply add a QPoint simply by reading the `/joint_state` position of the robot and add it to the list.

SPoints are defined following this structure:
```
POINT_NAME  X Y Z ANGLE
```
While QPoints are defined following this structure:
```
POINT_NAME  JOINT1 JOINT2 ... JOINTN
```
Be careful: the only separator between values is the space!!

## Action Definition
Actions are defined in the [actions_library](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/src/actions_library.py) file. It is a python file imported in the State Machine Node, and contains several action functions.
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

## Operation Definition
Operations are defined in a similar way as actions.
These simply call the actions with specific parameters defined in the code, so the user just have to select the operation without the need to insert any additional parameter.
These are the ones used in the Loop State, where the operator simply chooses an Operation from the list and launches it for a given number of times.
