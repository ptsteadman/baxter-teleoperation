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

banner = "WELCOME TO BAXTER"
DEFAULT_RATE = .05
IDLE_ANGLES = {
    'right_s0' : 0,
    'right_s1' : 0,
    'right_e0' : 0,
    'right_e1' : 0,
    'right_w0' : 0,
    'right_w1' : 0,
    'right_w2' : 0,
    'left_s0' : 0,
    'left_s1' : 0,
    'left_e0' : 0,
    'left_e1' : 0,
    'left_w0' : 0,
    'left_w1' : 0,
    'left_w2' : 0,
    'head_pan' : 0,
    'nod' : 0
    }
DEFAULT_IMAGE = 'on.png'

#commands
#start: self.thread.start()
#stop: self.exit = True
#start teleop: self.queue_state({'position_mode' : 'start_teleoperation', 'scenario_number' : 2})

class BaxterInterface(object):

    def __init__(self):
        ''' Initializes the interface to Baxter.  Baxter should already be enabled. '''
        # initialize console variables
        self.image_timer = None
        self.image_queue = deque()
        self.image_queue_lock = threading.Lock()
        self.motion_queue = deque()
        self.motion_queue_lock = threading.Lock()
        self.motion_timer = None
        self.mirrored = False
        self.user = 1
        self.thread = stopthread.StopThread(lambda : self.control_loop(None))
        self.state_queue = deque()
        self.state_queue_lock = threading.Lock()
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
        self.teleop = False
        # create and start the command loop
        #self.loop = multiprocessing.Process(target=self.control_loop,
                #args=(10,))
        #self.loop.start()
        self.exit = False
        #starting position
        self.idle_position()


        
    def control_loop(self, t):
        while not rospy.is_shutdown() and not self.exit:
            print "loop entered"

            # JOINTS: set baxter's joint angles based on current mode
            if self.teleop:
                joint_angles = get_joint_angles(self.user, self.tfBuffer, False, self.mirrored)
                if joint_angles is not None:
                    self.left_limb.set_joint_positions(joint_angles['left'])
                    self.right_limb.set_joint_positions(joint_angles['right'])
            
            
            #STATE
            #pop state queue if other queues are empty
            self.state_queue_lock.acquire()
            self.motion_queue_lock.acquire()
            self.image_queue_lock.acquire()
            empty_queues = len(self.motion_queue) == 0 and len(self.image_queue) == 0
            self.motion_queue_lock.release()
            self.image_queue_lock.release()
            #motion and image queues were released in case we need to add to them
            if empty and len(self.state_queue) > 0:
                #pop state
                next_state = self.state_queue.pop()
                #look for image data
                if 'image_mode' in next_state:
                    #load a csv file for images
                    if next_state['image_mode'] == 'load_csv_file':
                        self.load_images_file(next_state['image_filepath'])
                    #list of images with duration
                    elif next_state['image_mode'] == 'list':
                        for image in next_state['image_list']:
                            self.queue_image(image['filepath'], image['duration'])
                #look for position data                             
                if 'position_mode' in next_state:
                    self.teleop = False
                    #load a csv file for positions
                    if next_state['position_mode'] == 'load_csv_file':
                        self.load_position_file(filename)
                    #list of positions with duration
                    elif next_state['position_mode'] == 'list':
                        for motion in next_state['position_list']:
                            self.add_motion(motion["duration"], motion["angles"])
                    #start teleoperation
                    elif next_state['position_mode'] == 'teleoperation':
                       self.teleop = True
                       self.motion_timer = None  
                    #start teleop WITH transitions first
		            elif next_state['position_mode'] ==  'start_teleoperation':
		                self.start_teleoperation(next_state['scenario_number'])
		                self.state_queue.appendleft({'position_mode' : 'teleoperation'})
            self.state_queue_lock.release()       
                            
            
            #POSITION
            #motion queue checking
            self.motion_queue_lock.acquire()
            if self.motion_timer is not None:
                self.motion_timer = self.motion_timer + self.rate
            if len(self.motion_queue) > 0:
                #ready to publish new motion
                if self.motion_timer is None:
                   self.right_limb.set_joint_positions(self.motion_queue[0])
                   self.left_limb.set_joint_positions(self.motion_queue[0])
                   if 'head_pan' in self.motion_queue[0]:
                       self.head.set_pan(self.motion_queue[0]['head_pan'])
                   if 'head_nod' in self.motion_queue[0] and self.motion_queue[0] == 1:
                       self.head.command_nod()
                       
                   self.motion_timer = 0
                #motion expired (publish new motion next turn if queue not empty)
                elif self.motion_timer >= self.motion_queue[0]['duration']:
                    self.motion_queue.pop()
                    if len(self.motion_queue) == 0:
                       self.right_limb.set_joint_positions(IDLE_ANGLES)
                       self.left_limb.set_joint_positions(IDLE_ANGLES)
                       self.head.set_pan(0)
                       
                    self.motion_timer == None
            self.motion_queue_lock.release()
            
            #IMAGE
            #image queue check 
            self.image_queue_lock.acquire()
            if self.image_timer is not None:
                self.image_timer = self.image_timer + self.rate
            if len(self.image_queue) > 0:
            	#ready to publish new image
                if self.image_timer is None:
                    self.send_image(self.image_queue[0]['path'])
                    self.image_timer = 0
                #image expired (publish new image next turn if queue not empty)
                elif self.image_timer >= self.image_queue[0]['duration']:
                    self.image_queue.pop()
                    if len(self.image_queue) == 0:
                        self.send_image(DEFAULT_IMAGE)
                    self.image_timer = None
            self.image_queue_lock.release()
           
            time.sleep(self.rate)
        
        
        self.thread.stop()
        if self.exit:
            rospy.Shutdown()
        print "Rospy shutdown, exiting command loop."

    def set_rate(self, t):
        """ Sets the rate to send commands to the robot in ms """
        self.rate = rospy.Rate(t)
        
    def idle(self): 
        self.queue.put({"mode":"idle"})
    
    def standby(self): 
        self.queue_file("standby.csv")
        self.queue_image("on.png", 5)

        
    def start_teleoperation(self, transition):
        # first, execute transition
        if transition > 1:
            if transition == 2:
                self.queue_image('eyesClosed.png', 3)
                self.queue_image('eyesOpened.png', 1)
                self.queue_image('eyesClosed.png', .25)
                self.queue_image('eyesOpened.png', .25)
                self.queue_image('eyesClosed.png', .5)
                self.queue_image('eyesOpened.png', .15)
                self.queue_image("smile.png", .5)
                self.queue_image("eyesOpened.png", 3)
            elif transition == 3:
                self.queue_image("on with patrick.png", 3)


    
    def queue_image(self, new_image, duration):
        ''' Adds the image to a queue of images to be shown for a specified amount of time '''
        self.image_queue_lock.aquire()
        self.image_queue.append({'duration': duration * 1000, 'path': new_image })
        self.image_queue_lock.release()
        print "Image added to queue."


    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.
        @param path: path to the image file to load and send
        """
        img = cv2.imread('images/' + path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        # removed by alice
        #rospy.sleep(1)

    def interact(self):
        baxter = self
        code.interact(banner=banner, local=locals())
        
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
                self.add_motion(position_pieces[0], this_position_dict)
    
    def queue_state(self, dict):
        self.state_queue_lock.acquire()
        self.state_queue.append(dict)
        self.state_queue_lock.release()
     

   
    def idle_position(self):
         self.right_limb.set_joint_positions(IDLE_ANGLES)
         self.left_limb.set_joint_positions(IDLE_ANGLES)
         self.head.set_pan(IDLE_ANGLES['head_pan'])
         self.send_image(DEFAULT_IMAGE)

         
    def add_motion(self, duration, motion_dict):
        self.motion_queue_lock.acquire()
        self.motion_queue.append({"duration" : position_pieces[0], "positions" : this_position_dict}) 
        self.motion_queue_lock.release()      

if __name__ == '__main__':
    baxter = BaxterInterface()
    baxter.interact()
    

