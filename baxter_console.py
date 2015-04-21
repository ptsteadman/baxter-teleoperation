#!/usr/bin/env python

import code
from collections import deque
import time
import multiprocessing
import rospy
import tf2_ros
import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
import baxter_interface
import os
import sys
import argparse
import cv2
import cv_bridge
import threading
import stopthread
from sensor_msgs.msg import Image

BANNER = "WELCOME TO BAXTER"
DEFAULT_RATE = 10
IDLE_ANGLES = {
    'right_s0' : 0,
    'right_s1' : 0,
    'right_e0' : 0,
    'right_e1' : 0,
    'right_w0' : 0,
    'right_w1' : 0,
    'right_w2' : 0,
    'left_s0' : 3.14,
    'left_s1' : 0,
    'left_e0' : 0,
    'left_e1' : 3.14,
    'left_w0' : 0,
    'left_w1' : 0,
    'left_w2' : 0,
    'head_pan' : 0,
    'nod' : 0
    }
DEFAULT_IMAGE = 'images/eyesClosed.png'

class BaxterInterface(object):

    def __init__(self):
        ''' Initializes the interface to Baxter. '''
        # initialize console variables
        self.image_timer = None
        self.image_queue = deque()
        self.image_queue_lock = threading.Lock()
        self.motion_queue = deque()
        self.motion_queue_lock = threading.Lock()
        self.motion_timer = None
        self.mirrored = False
        self.user = 1
        self.state_queue = deque()
        self.state_queue_lock = threading.Lock()
        self.current_state = None
        self.exit = False

        # initialize the robot
        print("Initializing node... ")
        rospy.init_node("teleoperation")
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        
        def clean_shutdown():
            print("\nExiting...")
            if not init_state:
                print("Disabling robot...")
                rs.disable()
        rospy.on_shutdown(clean_shutdown)

        print("Enabling robot... ")
        rs.enable()

        # create interfaces to limbs and kinect tracking
        self.left_limb = baxter_interface.Limb('left')
        self.right_limb = baxter_interface.Limb('right')
        self.head = baxter_interface.Head()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(DEFAULT_RATE)

        #starting position
        self.idle()
        self.thread = stopthread.StopThread(lambda : self.control_loop())
        self.thread.start()

    def control_loop(self):
        while not rospy.is_shutdown():
            if self.exit:
                rospy.Shutdown()

            # STATE
            self.state_queue_lock.acquire()
            self.motion_queue_lock.acquire()
            self.image_queue_lock.acquire()
            empty_queues = len(self.motion_queue) == 0 and len(self.image_queue) == 0
            self.motion_queue_lock.release()
            self.image_queue_lock.release()
            # motion and image queues are released in case we need to add to them
            if empty_queues and len(self.state_queue) > 0:
                # pop state queue if image and state queues are empty/finished
                self.current_state = self.state_queue.pop()
                # look for image data
                if 'image_mode' in self.current_state:
                    # load a csv file for images
                    if self.current_state['image_mode'] == 'csv_file':
                        self.load_images_file(self.current_state['image_filepath'])
                    # list of images with duration
                    elif self.current_state['image_mode'] == 'list':
                        for image in self.current_state['image_list']:
                            self.queue_image(image['filepath'], image['duration'])
                else:
                    self.current_state['image_mode'] = "idle"
                # look for position data                             
                if 'position_mode' in self.current_state:
                    # load a csv file for positions
                    if self.current_state['position_mode'] == 'csv_file':
                        self.load_position_file(filename)
                    # list of positions with duration
                    elif self.current_state['position_mode'] == 'list':
                        for motion in self.current_state['position_list']:
                            self.queue_motion(motion["duration"], motion["angles"])
                    # start teleop WITH transition first
                    elif self.current_state['position_mode'] ==  'teleoperation_transition':
                        self.queue_transition(self.current_state['transition'])
                        self.state_queue.appendleft({'position_mode' : 'teleoperation'})
                else:
                    self.current_state['position_mode'] = "idle"
            self.state_queue_lock.release()       

            # JOINT POSITIONS
            if self.current_state['position_mode'] != "idle":
                if self.current_state['position_mode'] == "stopping":
                    self.right_limb.set_joint_positions(IDLE_ANGLES)
                    self.left_limb.set_joint_positions(IDLE_ANGLES)
                    self.head.set_pan(0)
                    self.current_state['position_mode'] = "idle"
                elif self.current_state['position_mode'] == "teleoperation":
                    # if the robot is being teleoperated, get kinect joint angles
                    self.motion_timer = None
                    joint_angles = get_joint_angles(self.user, self.tfBuffer, False, self.mirrored)
                    if joint_angles is not None:
                        self.left_limb.set_joint_positions(joint_angles['left'])
                        self.right_limb.set_joint_positions(joint_angles['right'])
                else:
                    # if the robot is not idle or teleoperated, check motion queue
                    self.motion_queue_lock.acquire()
                    if self.motion_timer is not None:
                        self.motion_timer = self.motion_timer + DEFAULT_RATE
                    if len(self.motion_queue) > 0:
                        #ready to publish new motion
                        if self.motion_timer is None:
                           self.right_limb.set_joint_positions(self.motion_queue[0])
                           self.left_limb.set_joint_positions(self.motion_queue[0])
                           if 'head_pan' in self.motion_queue[0]:
                               self.head.set_pan(self.motion_queue[0]['head_pan'])
                           # self.motion_queue[0] == 1 ?
                           if 'head_nod' in self.motion_queue[0] and self.motion_queue[0] == 1:
                               self.head.command_nod()
                           self.motion_timer = 0
                        #motion expired (publish new motion next turn if queue not empty)
                        elif self.motion_timer >= self.motion_queue[0]['duration']:
                            if self.motion_queue[0] != 0 and len(self.motion_queue) == 0:
                                # if duration is 0, hold position indefinitely
                                self.current_state["position_mode"] == "stopping"
                            self.motion_queue.pop()
                            self.motion_timer == None
                    self.motion_queue_lock.release()
            
            #IMAGES
            if self.current_state["image_mode"] is not "idle":
                if self.current_state["image_mode"] is "stopping":
                    self.send_image(DEFAULT_IMAGE)
                    self.image_queue.clear()
                    self.current_state["image_mode"] = "idle"
                else:
                    # update image timer/queue
                    self.image_queue_lock.acquire()
                    if self.image_timer is not None:
                        self.image_timer = self.image_timer + DEFAULT_RATE
                    if len(self.image_queue) > 0:
                        #ready to publish new image
                        if self.image_timer is None:
                            self.send_image(self.image_queue[0]['filepath'])
                            self.image_timer = 0
                        #image expired (publish new image next turn if queue not empty)
                        elif self.image_timer > self.image_queue[0]['duration']:
                            if self.image_queue[0]['duration'] != 0 and len(self.image_queue) == 0:
                                # if duration is 0, display image indefinitely
                                self.current_state["image_mode"] = "stopping"
                            self.image_queue.pop()
                            self.image_timer = None
                    self.image_queue_lock.release()
            
            self.rate.sleep()
        self.thread.stop()
        print "Rospy shutdown, exiting command loop."

    # User Functions

    def idle(self): 
        self.queue_state({"position_mode":"stopping", "image_mode": "stopping"})
    
    def standby(self): 
        self.queue_state({"image_mode":"list", "image_list":[{"duration":0, "filepath":"images/on.png"}]})
        # TODO: standby position

    def start_teleoperation(self, transition=1):
        if transition == 1:
            self.queue_state({"position_mode":"teleoperation"})
        else:
            self.queue_state({"position_mode":"teleoperation_transition","transition": transition})

    def queue_state(self, dict):
        self.state_queue_lock.acquire()
        self.state_queue.append(dict)
        self.state_queue_lock.release()
         
    
    # Internal Functions
    def queue_motion(self, duration, motion_dict):
        self.motion_queue_lock.acquire()
        self.motion_queue.append({"duration" : duration, "positions" : motion_dict}) 
        self.motion_queue_lock.release()      

    
    def queue_image(self, new_image, duration):
        ''' Adds the image to a queue of images to be shown for a specified amount of time '''
        self.image_queue_lock.acquire()
        self.image_queue.append({'duration': duration, 'filepath': new_image })
        self.image_queue_lock.release()

    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.
        @param path: path to the image file to load and send
        """
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        # removed by alice
        #rospy.sleep(1)


    def interact(self):
        baxter = self
        code.interact(banner=BANNER, local=locals())
        
    def load_image_file(self, filename):
        with open(filename).readlines() as filelines:
            keys = filelines[0].split(',')
            for image_line in filelines[1:]:
                this_image_dict = {}
                image_pieces = position_line.split(',')
                for i in range(0,len(keys)):
                    this_image_dict[key[i]] = position_pieces[i]
                self.queue_image(this_image_dict['filepath'], this_image_dict['duration'])
                
    def load_position_file(self, filename):
        with open(filename).readlines() as filelines:
            keys = filelines[0].split(',')
            for position_line in filelines[1:]:
                this_position_dict = {}
                position_pieces = position_line.split(',')
                for i in range(1,len(keys)):
                    this_position_dict[key[i]] = position_pieces[i]
                self.queue_motion(position_pieces[0], this_position_dict)
    
    def queue_transition(self, transition):
        if transition == 2:
            self.load_image_file("csv/transition2.csv")
        elif transition == 3:
            self.load_image_file("csv/transition3.csv")

if __name__ == '__main__':
    baxter = BaxterInterface()
    baxter.interact()
    

