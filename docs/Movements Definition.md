# Movements Definition
At the present time, the software is not programmed to automatically understand high-level commands such as "Assemble object" or "Take object".
Every instruction given to the robot must correspond to an operation already written down somewhere in an "hard-code" way.
This basically is what users usually do when programming a robot for a specific task: they write down in its code what it has to do according to the set-up where it is mounted on.

Because of this, we defined three main types of hard-code movement definitions:
- **POINT DEFINITION:** where the user simply adds a point coordinate to a list of points;
- **ACTION DEFINITION:** where the user defines a set of operations that compose a more complex instruction, i. e. to take a specific object one must specify where to move the end effector, how to open the gripper, how to take the object with the gripper, etc.
Simple actions are useful to compose more complex workflows, but are robot and set-up dependant;
- **OPERATION DEFINITION:** operations are a complete workflow composed of a set of simple actions. These are defined by calling the specific simple action in the correct moment of the workflow.

## Point Definition
The corresponding file is `xxx`. There is only one Points file, since points are appended to the file and used by selecting the point from the list.
Points are defined following this structure:
```
POINT_NAME  [coordinates]
```
The point coordinates are the positions of every joint, basically the `positions` vector which is sent in the JointTrajectory message.

This may be useful for example to define some interesting points like the _HOME_ point.

## Action Definition
The corresponding file is `xxx`. It is a python file imported in the State Machine Node, and contains several action functions.
Actions are defined in a parametric way, allowing the user to call a generic action with the parameters needed at that time.
To add a new action, simply add the corresponding function to the python file.

In this case the action name is the name of the function called.
The structure of an action is similar to this:
```
def move_from_A_to_B_and_returns(P, V, A, E):
  """ A is the point where the robot is currently.
  The function "write_trajectory" is a standard method of the State.
  Keep in mind that the trajectory message is passed as self.trajectory """
  P = point "B" vector
  V = velocities vector
  A = accelerations vector
  E = effort vector """
  
  self.write_trajectory(P, V, A, E)
  self.send_trajectory()
```

So, for example, a more complex action may call other simpler actions:

```
def take_object(self, P, V, A, E):
  """ A is the point where the robot is currently.
  The function "write_trajectory" is a standard method of the State."
  P = point "B" vector; is defined as a list of vectors
  V = velocities vector; is defined as a list of vectors
  A = accelerations vector; is defined as a list of vectors
  E = effort vector; is defined as a list of vectors """
  
  # sets control variables to default value
  end = False
  self.goal = False
  i = 1
  
  # sends the first position
  self.write_trajectory(P[0], V[0], A[0], E[0])
  self.send_trajectory()
  
  while end == False:
    # waits for feedback: if position reached, then it sends the second message
    # this is decided by the feedback callback, which returns a self.goal = True in this case
    self.trajSub = rospy.Subscriber('/feedback_topic', JointState, self.feedbackCallback, queue_size = 1)
    
    if self.goal == True:
      # if previous goal is reached, then it sends the new position message
      self.write_trajectory(P[i], V[i], A[i], E[i])
      self.send_trajectory()
      # checks if the execution of the whole movement is ended
      if i == len(P):
        end = True
        break
      else:
        i = i + 1
        # resets goal
        self.goal = False
```

## Operation Definition
The corresponding file is `xxx`.
Operations are defined in a similar way as actions.
These simply call the actions with specific parameters defined in the code, so the user just have to select the operation without the need to insert any additional parameter.












