# Gestures Node
Here the Gesture Node will be explained in more detail.
The corresponding file is `gesture_reader.py`.

## How does it work?
The Node is based on Deep Learning to recognize the hand gestures in the image frames obtained from the camera.
We fine-tuned an R-FCN Object Detector on our gestures dataset using the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection). 

We use a Kinect v2 camera as the image sensor, and read the frames by accessing the corresponding topics (i. e. `/kinect2/qhd/image_color_rect`).
To do so we use a `KinectCameraSubscriber` class object.

Note that we usually consider the `_rect` topics only because our cameras are **intrinsecally calibrated**.
Intrinsic calibration is needed to correct the distortion parameters of the camera, and correctly align the color information to the depth information obtained from the Kinect.
It is not a required step, but is a good practice to obtain good quality data!

After reading an image, this is sent to the frozen graph we trained on our dataset.
The graph is basically the detector object that reads the image as input and outputs a prediction dictionary.
The prediction contains:
- The _box coordinates_ of each hand gestures recognized in the picture;
- The _object class_ of each hand gesture;
- The _confidence score_ of each hand gesture.
We filter out predictions with confidence score **lower than 80%** as a safety measure, since Object Detectors are noisy (meaning that they tend to recognize a lot of objects in the image, usually wrong).

## Gestures
We defined a set of single-hand gestures as represented in the figure.

<p align="center">
  <img src="images/hands.png">
</p>

Since every gesture can be performed with the right or the left hand, and some gestures get confused with each other by the detector, we developed a "command code" to improve the performances.
This command code is based on the idea detailed in our paper: [Deep learning-based hand gesture recognition for collaborative robots](https://ieeexplore.ieee.org/abstract/document/8674634).
Basically, a valid gesture is composed by using **both hands at the same time**, allowing to create a more complex meaning.
In this case we do not use an hand as the anchor, but dynamically compose instructions according to the meaning of each single-hand gesture.

The meaning of the single-hand gestures is:
- gestures from (a) to (i) represent **numbers from 1 to 9**;
- gesture (j) is used as an interface command to **delete** the inserted instruction, allowing the user to re-enter the correct one;
- gesture (k) represents both the **number 0** and the **confirmation command**;
- gesture (l) has 4 variants, according to which hand is used and where the hand is pointing (right or left).
These are used as **directional gestures** to represent the increment (if the hand points right) or decrement (if the hand points left) of the position of the selected joint during the **Jog Mode**;
- gesture (m) represents the act to **interact with the system**. We use this gesture to access functionalities that request the user to open some file and modify it (using the keyboard);
- gesture (n) represents the **exit** command. It is used both as an interface command to exit the state and/or the confirmation panel, and to close the communication and/or the whole program;
- gesture (o) represents the **pause** command, to stop momentairly a robot execution and resume it in a later moment.

Thus, the composed commands are of two types: **Commands** and **Instructions**.

### Commands
Commands are statically defined and always represent the same meaning in the whole system.
These are:
- **CONFIRM** command: (k) + (k) gestures. It is used to confirm inserted commands/instructions and give the robot the signal to start or resume its movement;
- **DELETE** command: (j) + (j) gestures. It is used as the interface command to delete the command/instruction inserted before and allow the user to enter the correct one;
- **EXIT** command: (n) gesture. It has the same meaning described above, since it's performed with both hand by default;
- **PAUSE** command: (o) gesture. It has the same meaning described above, since it's performed with both hand by default;
- **OPEN FILE** command: (m) gesture. It has the same meaning described above, since it's performed with both hand by default.

### Instructions
Instructions are dynamically composed, thus allowing the user to create numbers or codes on-the-go. Some examples are:
- **CREATE SINGLE DIGIT NUMBER:** (k) + (a-i) gestures. Numbers are calculated considering the left gesture as the first digit and the right gesture as the second.
In this case the output is 0 + N = N;
- **CREATE TWO DIGIT NUMBER:** (a-i) + (a-i + k) gestures. Examples are: number 37 (c) + (g), number 90 (i) + (k), number 44 (d) + (d) and so on;
- **CREATE JOG INSTRUCTION:** Jog instructions are composed in a different way.
The numerical code is **100[Joint][1/0/-1]**. The first "100" is added by the system to distinguish the command from others.
The joint number is defined using a numerical gesture in the range from **1 to 9**. The 0 joint does not exists, and is used as an initialization value.
The final digit is defined according to the other hand gesture, that can be one of the (l) gestures or gesture (e):
    - If the hand points right it means that the user wants to increase the position of the joint. The value attached to this is **1** (i. e. 10041 means _increase joint 4 position_);
    - If the hand points left it means that the user wants to decrease the position of the joint. The value attached to this is also **1** but the whole instruction number is made negative (i. e. -10071 means _decrease joint 7 position_);
    - If the (e) gesture is used it means that the user wants the robot to stop in that position. The value attached to this is **0** (i. e. 10060 means _stop the robot in this position_).

### Adding custom gestures
If you want to add some other gestures defined by yourself, you can perform a fine-tuning of our model adding to the dataset your custom gestures too.
Download our model and follow the guidelines of the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) to find out the correct steps. If you need our dataset too, please contact us directly.

Other models can also be used: simply train your own (even in different frameworks if you prefer), froze them and load them in the code.

Remember to edit the State Machine and Gesture Node files to consider your new gestures too.
