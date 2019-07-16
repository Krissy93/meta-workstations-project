#!/usr/bin/env python

import os
import rospy
import tensorflow as tf
import time

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from utils import color
from state_machine_package.msg import commandRequest, commandResponse

##### MACROS DEFINITION

PKG_PATH = '/home/optolab/smach_ws/src/state_machine_package/'
# frozen graph path
PATH_TO_CKPT = PKG_PATH + 'network/RGB.pb'
# labelmap path
PATH_TO_LABELS = PKG_PATH + 'network/labelmap.pbtxt'
NUM_CLASSES = 29

class Gesture():
    ''' Object needed to simplify a bit the single hand prediction '''
    def __init__(self):
        self.type = None
        self.value = None
        self.hand = None


class Kinect():
    ''' Kinect object, it uses pylibfreenect2 as interface to get the frames.
    The original example was taken from the pylibfreenect2 github repository, at:
    https://github.com/r9y9/pylibfreenect2/blob/master/examples/selective_streams.py  '''

    def __init__(self, enable_rgb, enable_depth):
        ''' Init method called upon creation of Kinect object '''

        # according to the system, it loads the correct pipeline
        # and prints a log for the user
        try:
            from pylibfreenect2 import OpenGLPacketPipeline
            self.pipeline = OpenGLPacketPipeline()
        except:
            try:
                from pylibfreenect2 import OpenCLPacketPipeline
                self.pipeline = OpenCLPacketPipeline()
            except:
                from pylibfreenect2 import CpuPacketPipeline
                self.pipeline = CpuPacketPipeline()

        rospy.loginfo(color.BOLD + color.YELLOW + '-- PACKET PIPELINE: ' + str(type(self.pipeline).__name__) + ' --' + color.END)

        self.enable_rgb = enable_rgb
        self.enable_depth = enable_depth

        # creates the freenect2 device
        self.fn = Freenect2()
        # if no kinects are plugged in the system, it quits
        num_devices = self.fn.enumerateDevices()
        if num_devices == 0:
            rospy.loginfo(color.BOLD + color.RED + '-- ERROR: NO DEVICE CONNECTED!! --' + color.END)
            sys.exit(1)

        # otherwise it gets the first one available
        self.serial = self.fn.getDeviceSerialNumber(0)
        self.device = self.fn.openDevice(self.serial, pipeline=self.pipeline)

        # defines the streams to be acquired according to what the user wants
        types = 0
        if self.enable_rgb:
            types |= FrameType.Color
        if self.enable_depth:
            types |= (FrameType.Ir | FrameType.Depth)
        self.listener = SyncMultiFrameListener(types)

        # Register listeners
        if self.enable_rgb:
            self.device.setColorFrameListener(self.listener)
        if self.enable_depth:
            self.device.setIrAndDepthFrameListener(self.listener)

        if self.enable_rgb and self.enable_depth:
            self.device.start()
        else:
            self.device.startStreams(rgb=self.enable_rgb, depth=self.enable_depth)

        # NOTE: must be called after device.start()
        if self.enable_depth:
            self.registration = Registration(self.device.getIrCameraParams(), self.device.getColorCameraParams())

        # last number is bytes per pixel
        self.undistorted = Frame(512, 424, 4)
        self.registered = Frame(512, 424, 4)

    def acquire(self):
        ''' Acquisition method to trigger the Kinect to acquire new frames. '''

        # acquires a frame only if it's new
        frames = self.listener.waitForNewFrame()

        if self.enable_rgb:
            self.color = frames["color"]
            self.color_new = cv2.resize(self.color.asarray(), (int(1920 / 1), int(1080 / 1)))
            # The image obtained has a fourth dimension which is the alpha value
            # thus we have to remove it and take only the first three
            self.color_new = self.color_new[:,:,0:3]
            # the kinect sensor mirrors the images, so we have to flip them back
            self.color_new = cv2.flip(self.color_new, 1)
        if self.enable_depth:
            # these only have one dimension, we just need to convert them to arrays
            # if we want to perform detection on them
            self.depth = frames["depth"]
            self.depth_new = cv2.resize(self.depth.asarray() / 4500.)
            self.depth_new = cv2.flip(self.depth_new, 1)

            # ir stream, not needed for detection purposes right now

            #self.ir = frames["ir"]
            #self.ir_new = cv2.resize(self.ir.asarray() / 65535.)
            #self.ir_new = cv2.flip(self.ir_new, 1)

        ''' # This portion is needed if you want to print also the
            # registered (RGB+D) depth and the undistorted depth.

        if self.enable_rgb and self.enable_depth:
            self.registration.apply(self.color, self.depth, self.undistorted, self.registered)
            self.registered_new = self.registered.asarray(np.uint8)
            self.registered_new = cv2.flip(self.registered_new, 1)
        elif self.enable_depth:
            self.registration.undistortDepth(self.depth, self.undistorted)
            self.undistorted_new = cv2.resize(self.undistorted.asarray(np.float32) / 4500.)
            self.undistorted_new = cv2.flip(self.undistorted_new, 1) '''

        # do this anyway to release every acquired frame
        self.listener.release(frames)


class MessageUpdater():
    ''' This object performs the detection of hand gestures on frames obtained
    from the Kinect object. It translates the gestures as two-hand ones and sends them
    upon request on the corresponding response topic. '''

    def __init__(self):
        ''' Init method called upon creation of Kinect object '''

        # initializes some useful variables
        self.request = commandRequest()
        self.request.request_number = 0
        self.request.request_type = 'change_state'
        self.response = commandResponse()
        self.sub = 0
        self.pub = rospy.Publisher('/command_response', commandResponse, queue_size = 1)
        # may be useful to publish the detected frames on a ros topic, in order to save the rosbag for later
        # self.pub2 = rospy.Publisher('/detections', Image, queue_size = 1)
        self.received = False
        self.correct = False
        self.quit = False
        self.detection_list = []
        self.command = 'NONE'

        # initializes the Kinect object here
        # first parameter is the enabling RGB, second is the enabling DEPTH and IR
        self.kinect = Kinect(True, False)

        # loads the labelmap and classes definition into memory
        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(self.label_map, max_num_classes=NUM_CLASSES,use_display_name=True)
        self.category_index = label_map_util.create_category_index(self.categories)
        # loads the model into memory and saves it. This requires some time and slows
        # the first detection, which takes around 3 seconds
        rospy.loginfo(color.BOLD + color.YELLOW + '-- TF GRAPH INITIALIZATION --' + color.END)
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(fid.read())
        with tf.Graph().as_default() as graph:
            tf.import_graph_def(graph_def, name='prefix')
        # finally it saves it as a class variable
        self.detection_graph = graph

    def messageCallback(self, msg):
        ''' Callback function to correctly set the request and response numbers '''

        # if new request is equal to the old request number plus one, then it's
        # a new request and we have to elaborate a response!
        if msg.request_number == self.request.request_number + 1:
            self.request = msg
            rospy.loginfo(color.BOLD + color.YELLOW + 'RECEIVED REQUEST NUMBER: ' + str(self.request.request_number) + color.END)
            self.received = True

    def detect(self, sess):
        ''' Method to perform the actual inference on frames acquired from the
        Kinect object triggerd outside this function. To correctly display the frames
        the method returns the detected image and this is used for display outside the method '''

        # we want to use the flipped and resized numpy array
        image = self.kinect.color_new
        # the numpy array of the image is expanded (1 dimension added needed for the net)
        image_np_expanded = np.expand_dims(image, axis=0)
        # extract image tensor
        image_tensor = self.detection_graph.get_tensor_by_name('prefix/image_tensor:0')
        # extract detection boxes
        boxes = self.detection_graph.get_tensor_by_name('prefix/detection_boxes:0')
        # extract detection scores
        scores = self.detection_graph.get_tensor_by_name('prefix/detection_scores:0')
        # extract detection classes
        classes = self.detection_graph.get_tensor_by_name('prefix/detection_classes:0')
        # extract number of detectionsd
        num_detections = self.detection_graph.get_tensor_by_name('prefix/num_detections:0')

        # the actual detection is performed here upon calling the session
        # to achieve real-time performances it is fundamental to define the session
        # outside any kind of loop and call the method in the loop afterwards
        start = time.time()
        (boxes, scores, classes, num_detections) = sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        #b = np.squeeze(boxes)
        #c = np.squeeze(classes).astype(np.int32)
        #s = np.squeeze(scores)
        b = np.array([[0., 0., 0., 0.]])
        c = np.array([1]).astype(np.int32)
        s = np.array([0.])
        # after inference is done, it performs the 2 hands gesture elaboration
        # and sends the command to the State Machine node.
        # this is part of the inference pipeline so the end time is calculated afterwards
        self.received = True
        if self.received == True and num_detections > 0:
            s, c, b = self.two_hands_gesture(boxes, classes, scores, num_detections)
            if len(self.detection_list) > 0:
                # I have at least one element in the list
                if (self.detection_list.count(self.detection_list[0]) == len(self.detection_list)):
                    if (len(self.detection_list) == 10):
                        # send message if each element is the same and we have exactly 3 elements
                        # else it does nothing
                        self.correct = True
                        self.detection_list = []
                else:
                    # else it means that we don't have each element equal
                    # like [A,B] results in 1 not 2 (which is the length of the list)
                    # thus we need to empty the list and start again
                    self.detection_list = []

        else: ## controlla received
            self.received = False
            self.correct = False
            # empty the list of consequent detections if connection interrupted
            self.detection_list = []

        end = time.time()
        inference = format(end-start, '.3f')

        # draws boxes and labels on the image. These are the detected ones not the filtered ones
        vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            b,
            c,
            s,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=4)

        # opencv here gets an error so we just copy the obtained numpy array image in order
        # to draw on it some text.
        A = image.copy()
        # the structure is: image, string of text, position from the top-left angle, font, size, BGR value of txt color, thickness, graphic
        cv2.putText(A, 'INFERENCE TIME: ' + str(inference) + ' SEC', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (80, 65, 242), 3, cv2.LINE_AA)
        cv2.putText(A, 'LAST REQUEST RECEIVED: ' + str(self.request.request_number), (20,930), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (221, 224, 27), 3, cv2.LINE_AA)
        cv2.putText(A, 'LAST RESPONSE SENT: ' + str(self.response.command), (20,980), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (35, 227, 25), 3, cv2.LINE_AA)
        # finally we display the image on a new window
        cv2.imshow('object detection', A)

    def single_hand_gesture(self, value):
        """ Fills in the gesture parameters. These are:
        Type = Command, Number, Direction
        Value = simply the value of the command
        Hand = right or left, None if two hand command """

        gesture_handler = Gesture()
        # these five are the basic ones
        if (value == 28):
            # EXIT command
            gesture_handler.type = 'command'
            gesture_handler.value = -2
            gesture_handler.hand = 'both'
        elif (value == 29):
            # PAUSE command
            gesture_handler.type = 'command'
            gesture_handler.value = -4
            gesture_handler.hand = 'both'
        elif (value == 27):
            # OPEN FILE command
            gesture_handler.type = 'command'
            gesture_handler.value = -5
            gesture_handler.hand = 'both'
        elif (value >= 1) and (value <= 22):
            # these are all the numerical commands including punch gestures and span gestures
            if (value % 2) > 0:
                # if odd number is the right hand one
                gesture_handler.value = value / 2
                gesture_handler.hand = 'right'
            else:
                # else it's the even one so I have to subtract 1 to obtain the correct numerical value
                gesture_handler.value = (value / 2) - 1
                gesture_handler.hand = 'left'
            if (value >= 1) and (value <= 20):
                # includes punches 'cause these are the zeros
                gesture_handler.type = 'number'
            else:
                # spans only
                gesture_handler.type = 'command'
        elif (value == 23) or (value == 26):
            # hand pointing right
            gesture_handler.type = 'direction'
            gesture_handler.value = 1
            if value == 23:
                gesture_handler.hand = 'left'
            else:
                gesture_handler.hand = 'right'
        elif (value == 24) or (value == 25):
            # hand pointing left
            gesture_handler.type = 'direction'
            gesture_handler.value = -1
            if value == 24:
                gesture_handler.hand = 'left'
            else:
                gesture_handler.hand = 'right'

        return gesture_handler

    def two_hands_gesture(self, boxes, classes, scores, num_detections):
        """ Function to compose gestures as two hand gestures.
        b = boxes array obtained from prediction dictionary
        s = confidence scores array obtained from prediction dictionary
        c = classes array obtained from prediction dictionary """

        # gets the values from the lists
        b = np.squeeze(boxes)[0:int(num_detections)]
        s = np.squeeze(scores)[0:int(num_detections)]
        c = np.squeeze(classes)[0:int(num_detections)]
        c = c.astype(int)

        # initializes the variables
        N = -10
        R = None
        L = None
        D = None
        # check the type of request
        type = self.request.request_type

        # if more than 2 detections, chooses only the 2 with max scores
        if int(num_detections) > 2:
            # chooses the first max one
            first = max(s)
            # deletes it from a copy of the array
            ss = np.delete(s, np.argmax(s))
            # repeats to find the second max one
            second = max(ss)
            # gets indexes of first and second score values
            x = np.where(s == first)[0][0]
            y = np.where(s == second)[0][0]
            # selects boxes, scores and classes according to the indexes obtained
            s = np.array([s[x], s[y]])
            c = np.array([c[x], c[y]])
            b = np.array([b[x], b[y]])

        # at this point only 1 or 2 elements are present in the arrays!

        # gets the values of each gesture
        for i in range(0,len(c)):
            g = self.single_hand_gesture(c[i])
            if g.hand == 'right':
                R = g.value
            elif g.hand == 'left':
                L = g.value

            if g.type == 'direction':
                D = g.value

            if g.type == 'command':
                N = g.value

        if (R >= 0) and (L >= 0) and (type != 'jog_command'):
            # if I have both R and L and it is not a jog command, then it is
            # a composed instruction
            if (R == 0) and (L == 0):
                # Two punches, meaning CONFIRM instruction
                N = 0
            elif (R == 10) and (L == 10):
                # Two span gestures, meaning CANCEL instruction
                N = -3
            else:
                # else is a composed number, the zero gets deleted upon casting to int
                N = str(R) + str(L)
                N = int(N)

        elif (type == 'jog_command'):
            # Jog command
            # cases where I have joint and direction 1/-1
            if (R != None) and (L == None):
                # if I have some value in R but not in L it means that I have a joint + direction command
                J = R
            elif (L != None) and (R == None):
                # if I have some value in L but not in R it means that I have a joint + direction command
                J = L
            # cases where I have a 5 somewhere
            elif (R != None) and (L != None):
                if (R == 5) and (L == 5):
                    # special case when I have both the joint and the direction equal to five gesture
                    D = 0
                    J = 5
                elif (R == 5) and (L != 5):
                    # only R is equal to 5, meaning that R is not the joint value
                    D = 0
                    J = L
                elif (R != 5) and (L == 5):
                    # only L is equal to 5, meaning that L is not the joint value
                    D = 0
                    J = R

            N = '100' + str(J) + str(abs(D))
            N = int(N)
            if (D < 0):
                # if D is negative change the sign
                N = - N

        # rospy.loginfo(color.BOLD + color.RED + str(N) + color.END)
        # check for correctness
        if (N >= 0) or (N >= -5 and N <= -2) or (N >= -10100 and N <= -10000):
            # this is only a check to assure the N obtained is valid
            # if so performes the updates
            self.command = N
            self.detection_list.append([R, L, D, N])
            # rospy.loginfo(color.BOLD + color.YELLOW + '-- COMMAND: ' + str(N) + ' --' + color.END)
        else:
            rospy.loginfo(color.BOLD + color.RED + '-- WARNING: INVALID GESTURE --' + color.END)

        return s, c, b


    def execute(self, sess):
        ''' This method is the one where the detection happens if a request
        has been received. If no request has been received, the acquired frame
        is displayed as it is '''

        # initializes variables
        self.correct = False
        self.received = False
        # waits for the request
        while self.correct == False:
            # triggers the kinect to acquire a frame
            self.kinect.acquire()
            # checks the request
            self.sub = rospy.Subscriber('/command_request', commandRequest, self.messageCallback, queue_size=1)
            # if new request has been correctly received, calls the detection function
            # the kinect frame is stored in self.kinect.color_new and similar variables
            self.detect(sess)
            if self.correct == True:
                self.response.request_number = self.request.request_number
                self.response.request_type = self.request.request_type
                self.response.command = self.command
                self.pub.publish(self.response)
                rospy.loginfo(color.BOLD + color.YELLOW + '-- COMMAND SENT: ' + str(self.response.command) + ' --' + color.END)

            # portion needed to correctly close the opencv image window
            if cv2.waitKey(25) == ord('q'):
                cv2.destroyAllWindows()
                self.quit = True
                break

def myhook():
    rospy.loginfo(color.BOLD + color.RED + '\n -- KEYBOARD INTERRUPT, SHUTTING DOWN --' + color.END)

def main():
    rospy.init_node('gestures_node')
    updater = MessageUpdater()
    # this structure is FUNDAMENTAL: the session is defined at first and inside
    # this definition we can call every loop we want. By doing this we are using
    # the same model without constantly loading it into memory!!!
    with tf.Session(graph=updater.detection_graph) as sess:
        while not rospy.is_shutdown():
            updater.execute(sess)
            if updater.quit == True:
                break
        # upon exiting the loop, we close the device and quit the program
        rospy.loginfo(color.BOLD + color.RED + '\n -- CLOSING DEVICE... --' + color.END)
        updater.kinect.device.stop()
        updater.kinect.device.close()

    rospy.on_shutdown(myhook)

if __name__ == '__main__':
    main()
