# Gestures Node
Here the Gesture Node will be explained in more detail.
The corresponding file is `gesture_reader.py`.

## How does it work?
The Node is based on Deep Learning to recognize the hand gestures in the image frames obtained from the camera.
We fine-tuned an R-FCN Object Detector on our gestures dataset using the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). 

We use a Kinect v2 camera as the image sensor. To read the frames we can:
- access the corresponding topics (i. e. `/kinect2/qhd/image_color_rect`) using [kinect2_bridge](https://github.com/code-iai/iai_kinect2) and [libfreenect2](https://github.com/OpenKinect/libfreenect2)
- use [pylibfreenect2](https://github.com/r9y9/pylibfreenect2) and directly access the Kinect without using a ROS bridge to do it.

We choose to use the latter in this version to reduce the amount of ROS nodes and improve speed.

**NOTE:** when using `kinect2_bridge` we usually consider the `_rect` topics because our cameras are **intrinsecally calibrated**.
Intrinsic calibration is needed to correct the distortion parameters of the camera, and correctly align the color information to the depth information obtained from the Kinect (if you need it).
It is not a required step, but is a good practice to obtain good quality data!

After reading an image, this is sent to the frozen graph we trained on our dataset.
The graph is basically the detector object that reads the image as input and outputs a prediction dictionary.
The prediction contains:
- The _box coordinates_ of each hand gestures recognized in the picture;
- The _object class_ of each hand gesture;
- The _confidence score_ of each hand gesture.

We filter out predictions as a safety measure, since Object Detectors are noisy (meaning that they tend to recognize a lot of objects in the image, usually wrong). The filtering procedure is the following:
- if we read more than 2 gestures, only the ones with maximum score are retained;
- we check the gesture labels obtained according to the `/command_request` received from the State Machine node. This also filters out same hand gestures automatically;
- if the pair survived the filtering, we store its numerical command value in a vector. If this vector is filled up with the same unique numerical values up to a certain threshold (good values are from 7 to 10), the composed gesture is valid and is written in the `/command_response` topic. Basically this assures that the same gesture is read from 7 to 10 times consecutively without a wrong gesture reading in-between.

## Gestures
We defined a set of single-hand gestures as represented in figure 2 of [our paper](https://doi.org/10.1016/j.rcim.2020.102085).

Since every gesture can be performed with the right or the left hand, and some gestures get confused with each other by the detector, we developed a "command code" to improve the performances.
This command code is based on the idea detailed in our papers: [Deep learning-based hand gesture recognition for collaborative robots](https://ieeexplore.ieee.org/abstract/document/8674634) and [Hand Gesture Recognition for Collaborative Workstations: A Smart Command System Prototype](https://link.springer.com/chapter/10.1007%2F978-3-030-30754-7_33).
Basically, a valid gesture is composed by using **both hands at the same time**, allowing to create a more complex meaning.
In this case we do not use an hand as the anchor, but dynamically compose instructions according to the meaning of each single-hand gesture.

The meaning of the single-hand gestures is:
- gestures from (a) to (i) represent **numbers from 1 to 9**;
- gesture (j) represents both the **number 0** and the **confirmation command**;
- gesture (k) is used as an interface command to **delete** the inserted instruction, allowing the user to re-enter the correct one;
- gesture (l) has 4 variants, according to which hand is used and where the hand is pointing (right or left).
These are used as **directional gestures** to represent the increment (if the hand points right) or decrement (if the hand points left) of the position of the selected joint during the **Jog Mode**;
- gesture (m) represents the act to **interact with the system**. We use this gesture to access functionalities that request the user to open some file and modify it (using the keyboard);
- gesture (n) represents the **exit** command. It is used both as an interface command to exit the state and/or the confirmation panel, and to close the communication and/or the whole program;
- gesture (o) represents the **pause** command, to stop momentairly a robot execution and resume it in a later moment.

Thus, the composed commands are of two types: **Static Commands** and **Parametric Commands**.

### Static Commands
Static Commands are statically defined and always represent the same meaning in the whole system.
These are:
- **CONFIRM** command: (j) + (j) gestures. It is used to confirm inserted commands/instructions and give the robot the signal to start or resume its movement;
- **DELETE** command: (k) + (k) gestures. It is used as the interface command to delete the command/instruction inserted before and allow the user to enter the correct one;
- **EXIT** command: (n) gesture. It has the same meaning described above, since it's performed with both hand by default;
- **PAUSE** command: (o) gesture. It has the same meaning described above, since it's performed with both hand by default;
- **OPEN FILE** command: (m) gesture. It has the same meaning described above, since it's performed with both hand by default.

### Parametric Commands
Parametric Commands are dynamically composed, thus allowing the user to create numbers or codes on-the-go. Some examples are:
- **CREATE SINGLE DIGIT NUMBER:** (j) + (a-i) gestures. Numbers are calculated considering the left gesture as the first digit and the right gesture as the second.
In this case the output is 0 + N = N;
- **CREATE TWO DIGIT NUMBER:** (a-i) + (a-i + j) gestures. Examples are: number 37 (c) + (g), number 90 (i) + (j), number 44 (d) + (d) and so on;
- **CREATE JOG INSTRUCTION:** Jog instructions are composed in a different way.
The numerical code is **100[Joint][1/0/-1]**. The first "100" is added by the system to distinguish the command from others.
The joint number is defined using a numerical gesture in the range from **1 to 9**. The 0 joint does not exists, and is used as an initialization value.
The final digit is defined according to the other hand gesture, that can be one of the (l) gestures or gesture (e):
    - If the hand points right it means that the user wants to increase the position of the joint. The value attached to this is **1** (i. e. 10041 means _increase joint 4 position_);
    - If the hand points left it means that the user wants to decrease the position of the joint. The value attached to this is also **1** but the whole instruction number is made negative (i. e. -10071 means _decrease joint 7 position_);
    - If the (e) gesture is used it means that the user wants the robot to stop in that position. The value attached to this is **0** (i. e. 10060 means _stop the robot in this position_).

### Adding custom gestures
If you want to add some other gestures defined by yourself, you can perform a fine-tuning of our model adding to the dataset your custom gestures too.
Download our model and follow the guidelines of the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) to find out the correct steps. You can find our dataset [here](https://data.mendeley.com/datasets/ndrczc35bt/2) and, if you use it for your research, please cite it:

```
@article{HANDS_dataset,
title = {HANDS: an RGB-D dataset of static hand-gestures for human-robot interaction},
journal = {Data in Brief},
volume = {35},
pages = {106791},
year = {2021},
issn = {2352-3409},
doi = {https://doi.org/10.1016/j.dib.2021.106791},
url = {https://www.sciencedirect.com/science/article/pii/S2352340921000755},
author = {Cristina Nuzzi and Simone Pasinetti and Roberto Pagani and Gabriele Coffetti and Giovanna Sansoni}}
```

Other models can also be used: simply train your own (even in different frameworks if you prefer), froze them and load them in the code.

Remember to edit the State Machine and Gesture Node files to consider your new gestures too.

## How to use it?
Two versions are available: one that uses images as input to read the hand-gestures and send a command to the State Machine, one that bypasses this step and prompts the user to type a numerical command directly. The second version is usually used as a debug method to check if the transitions between the states are correct and basically check rapidly if everything works.

To use them, simply open two terminals. In the first one type:
```
roslaunch state_machine_package launch_state_machine.launch
```
which launches [this](https://github.com/Krissy93/meta-workstations-project/blob/master/state_machine_package/launch/launch_state_machine.launch) file. This starts the state machine and the SMACH viewer at the same time.

In the second terminal type:
```
roslaunch state_machine_package keyboard_gestures_node.launch
```
to launch the Keyboard Node, or
```
roslaunch state_machine_package hand_gestures_node.launch
```
to launch the Gestures Node.
