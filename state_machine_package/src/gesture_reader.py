#!/usr/bin/env python

import numpy as np
import os
import sys
import tensorflow as tf
import cv2
import freenect

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from state_machine_package.msg import commandRequest, commandResponse
import rospy

class Gesture():
    def __init__(self):
        self.type = None
        self.value = None
        self.hand = None

class KinectDataSubscriber:
    """ Holds the most up to date """
    def __init__(self):
        self.bridge = CvBridge()
        self.color_sub = rospy.Subscriber(COLOR_IMAGE_TOPIC,
                                          Image,
                                          self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(DEPTH_MAP_TOPIC,
                                          Image,
                                          self.depth_callback, queue_size=1)

        # data containers and its mutexes
        self.color_image = None
        self.depth_image = None
        self.color_mutex = Lock()
        self.depth_mutex = Lock()

    def color_callback(self, data):
        """ Called whenever color data is available. """
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.color_mutex.acquire()
        self.color_image = np.array(cv_image, dtype=np.uint8)
        self.color_mutex.release()

    def depth_callback(self, data):
        """ Called whenever depth data is available. """
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.depth_mutex.acquire()
        self.depth_image = np.array(cv_image, dtype=np.uint16)
        self.depth_mutex.release()

class MessageUpdater():
    def __init__(self):
        # initializes Kinect object
        self.kinect = KinectDataSubscriber()
        # initializes some usefule variables
        self.request = commandRequest()
        self.request.request_number = 1
        self.request.request_type = 'change_state'
        self.response = commandResponse()
        self.sub = 0
        self.pub = rospy.Publisher('/command_response', commandResponse, queue_size = 1)
        #self.rate = rospy.Rate(10)
        self.received = False
        self.correct = False
        # camera
        '''self.cap = cv2.VideoCapture(0)  # Change only if you have more than one webcams
        self.cap.set(10,0.6)
        self.cap.set(11,0.6)
        self.cap.set(12,0.65)
        self.cap.set(13,0.65)'''

        '''cap,_ = freenect.sync_get_video()
        cap = cv2.cvtColor(cap,cv2.COLOR_RGB2BGR)'''

        # model initialization
        PATH_TO_CKPT = 'network/frozen_RGB_graph.pb'
        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = 'network/labelmap.pbtxt'
        NUM_CLASSES = 29
        # Loading label map
        self.label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        self.categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        # Load a (frozen) Tensorflow model into memory.
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')


    def detect(self): ###############################
        # gets the frame from the Kinect
        self.kinect.color_mutex.acquire()
        if (data.color_image is not None):
            image_np = data.color_image.copy()
            data.color_mutex.release()
            # Detection
            with self.detection_graph.as_default():
                with tf.Session(graph=self.detection_graph) as sess:
                    # Read frame from camera
                    #ret, image_np = self.cap.read()
                    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                    image_np_expanded = np.expand_dims(image_np, axis=0)
                    # Extract image tensor
                    image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                    # Extract detection boxes
                    boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                    # Extract detection scores
                    scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                    # Extract detection classes
                    classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                    # Extract number of detectionsd
                    num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
                    # Actual detection.
                    (boxes, scores, classes, num_detections) = sess.run(
                        [boxes, scores, classes, num_detections],
                        feed_dict={image_tensor: image_np_expanded})

                    # Visualization of the results of a detection.
                    vis_util.visualize_boxes_and_labels_on_image_array(
                        image_np,
                        np.squeeze(boxes),
                        np.squeeze(classes).astype(np.int32),
                        np.squeeze(scores),
                        category_index,
                        use_normalized_coordinates=True,
                        line_thickness=8)

                    # Display output
                    cv2.imshow('object detection', cv2.resize(image_np, (800, 600)))
                    return boxes, classes, scores, num_detections
        else:
            data.color_mutex.release()


    def single_hand_gesture(value):
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

        # initializes the variables
        N = None
        R = None
        L = None
        D = None
        type = None

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

        # check the type of request
        self.request.request_type = type

        # gets the values of each gesture
        for i in range(0,len(c)):
            g = single_hand_gesture(c[i])
            if g.hand == 'right':
                R = g.value
            elif g.hand == 'left':
                L = g.value

            if g.type == 'direction':
                D = g.value

            if g.type == 'command':
                N = g.value

        if (R not None) and (L not None) and (type not 'jog_command'):
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
        if (type == 'jog_command'):
            # Jog command
            # cases where I have joint and direction 1/-1
            if (R not None) and (L == None):
                # if I have some value in R but not in L it means that I have a joint + direction command
                J = R
            elif (L not None) and (R == None):
                # if I have some value in L but not in R it means that I have a joint + direction command
                J = L
            # cases where I have a 5 somewhere
            elif (R not None) and (L not None):
                if (R == 5) and (L == 5):
                    # special case when I have both the joint and the direction equal to five gesture
                    D = 0
                    J = 5
                elif (R == 5) and (L not 5):
                    # only R is equal to 5, meaning that R is not the joint value
                    D = 0
                    J = L
                elif (R not 5) and (L == 5):
                    # only L is equal to 5, meaning that L is not the joint value
                    D = 0
                    J = R

            N = '100' + str(J) + str(abs(D))
            N = int(N)
            if (D < 0):
                # if D is negative change the sign
                N = - N

        if (abs(N) > 0 and abs(N) < 10091):
            # this is only a check to assure the N obtained is valid
            # if so, sends the response with updated parameters
            self.response.request_number = self.request.request_number
            self.response.request_type = self.request.request_type
            self.response.command = N
            self.pub.publish(self.response)
            rospy.loginfo(color.BOLD + color.YELLOW + '-- COMMAND SENT --' + color.END)
            self.correct = True
        else:
            rospy.loginfo(color.BOLD + color.RED + '-- WARNING: INVALID GESTURE --' + color.END)
            self.correct = False


    def messageCallback(self, msg):
        # check if new request has been received
        # according to the request number
        # if new request has number different from the old request number, then it's
        # a new request and I have to elaborate a response!
        if msg.request_number != self.request.request_number:
            self.request = msg
            rospy.loginfo(color.BOLD + color.YELLOW + 'RECEIVED REQUEST NUMBER: ' + str(self.request.request_number) + color.END)
            self.received = True


    def execute(self):
        # initializes variables
        self.correct = False
        self.received = False
        # waits for the request
        while self.correct == False:
            self.sub = rospy.Subscriber('/command_request', commandRequest, self.messageCallback, queue_size=1)
            # if request received, it tries to translate the gesture
            if self.received == True:
                # if new request has been correctly received, calls the detection function
                boxes, classes, scores, num_detections = self.detect()
                # after this, obtains the command gesture and sends it
                self.two_hands_gesture(boxes, classes, scores, num_detections)
                print self.response
            else:
                self.received = False
                self.correct = False

def videostream():
    # Define the video stream
    cap = cv2.VideoCapture(0)  # Change only if you have more than one webcams
    cap.set(10,0.6)
    cap.set(11,0.6)
    cap.set(12,0.65)
    cap.set(13,0.65)
    '''
    0 - CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds or video capture timestamp.
    x - CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
    x - CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file: 0 - start of the film, 1 - end of the film.
    3 - CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
    4 - CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
    5 - CV_CAP_PROP_FPS Frame rate.
    6 - CV_CAP_PROP_FOURCC 4-character code of codec.
    x - CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
    8 - CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
    9 - CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
    10- CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
    11- CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
    12- CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
    13- CV_CAP_PROP_HUE Hue of the image (only for cameras).
    x - CV_CAP_PROP_GAIN Gain of the image (only for cameras).
    15- CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
    16- CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
    x - CV_CAP_PROP_WHITE_BALANCE_U The U value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
    x - CV_CAP_PROP_WHITE_BALANCE_V The V value of the whitebalance setting (note: only supported by DC1394 v 2.x backend currently)
    x - CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)
    x - CV_CAP_PROP_ISO_SPEED The ISO speed of the camera (note: only supported by DC1394 v 2.x backend currently)
    x - CV_CAP_PROP_BUFFERSIZE Amount of frames stored in internal buffer memory (note: only supported by DC1394 v 2.x backend currently)
    '''
    return cap


def myhook():
    print(color.BOLD + color.RED + '\n -- KEYBOARD INTERRUPT, SHUTTING DOWN --' + color.END)

def main():
    rospy.init_node('gestures_node')
    updater = MessageUpdater()
    #cap = videostream()

    while not rospy.is_shutdown():
        updater.execute()

    rospy.on_shutdown(myhook)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
