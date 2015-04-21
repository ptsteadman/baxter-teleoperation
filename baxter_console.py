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

banner = "WELCOME TO BAXTER"
DEFAULT_RATE = 1000

class BaxterInterface(object):

    def __init__(self):
        # initialize variables
        self.state = dict()
        self.state["mode"] = "idle"
        self.image_timer = None
        self.image_queue = deque()
        self.motion_queue = deque()
        self.motion_timer = None
        self.queue = multiprocessing.Queue()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)
        self.rate = rospy.Rate(DEFAULT_RATE)
        self.mirrored = False
        self.user = 1

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

        self.left_limb = baxter_interface.Limb('left')
        self.right_limb = baxter_interface.Limb('right')

        # create and start the command loop
        self.loop = multiprocessing.Process(target=self.control_loop,
                args=(10,))
        self.loop.start()
        
    def set_rate(self, t):
        """ Sets the rate to send commands to the robot in ms """
        self.rate = rospy.Rate(t)

    def control_loop(self, t):
        while not rospy.is_shutdown():
            # USER COMMANDS: empty the queue of console commands
            while not self.queue.empty():
                command = self.queue.get()
                self.state[command['k']] = command['v']

            # JOINTS: set baxter's joint angles based on current mode
            if self.state["mode"] == "idle":
                self.left_limb.set_joint_positions(IDLE_ANGLES['left'])
                self.right_limb.set_joint_positions(IDLE_ANGLES['right'])
            if self.state["mode"] == "file_playback":
                # set joint positions according to csv file/timer
                pass
            if self.state["mode"] == "teleoperated":
                joint_angles = get_joint_angles(self.user, self.tfBuffer, False, self.mirrored)
                if joint_angles is not None:
                    self.left_limb.set_joint_positions(joint_angles['left'])
                    self.right_limb.set_joint_positions(joint_angles['right'])

            # SCREEN IMAGES 
            if self.image_timer is not None:
                self.image_timer = self.image_timer + self.rate
            if len(self.image_queue) > 0:
                if self.image_timer is None:
                    self.send_image(self.image_queue[0]['path'])
                    self.image_timer = 0
                elif self.image_timer > self.image_queue[0]['duration']:
                    self.image_queue.pop()
                    if len(self.image_queue) > 0:
                        self.send_image(self.image_queue[0]['path'])
                        self.image_timer = 0
                    if len(self.image_queue) == 0:
                        self.send_image('default.jpg')
                        self.image_timer = None

            rate.sleep()
        print "Rospy shutdown, exiting command loop."
        
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
        self.queue.put({"mode":"teleoperated"})

    def stop_teleoperation(self):
        self.queue.put({"mode":"idle"})

    def queue_file(self, filename):
        ''' Play a csv file of joint angles '''
        # read file
        # for row in file, add position to queue
    
    def queue_image(self, new_image, duration):
        ''' Adds the image to a queue of images to be shown for a specified amount of time '''
        self.image_queue.append({'duration': duration * 1000, 'path': new_image })
        print "Image added to queue."
            
    def nod(self):
        print "baxter nods"
        self.__head.command_nod()

    def send_image(self, path):
        """
        Send the image located at the specified path to the head
        display on Baxter.
        @param path: path to the image file to load and send
        """
        img = cv2.imread("images\\" + path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        # removed by alice
        #rospy.sleep(1)

    def interact(self):
        baxter = self
        code.interact(banner=banner, local=locals())
    def load_position_file(self, filename):
        with open(filename).readlines() as filelines:
            keys = filelines[0].split(',')
            for position_line in filelines[1:]:
                this_position = {}
                for i in range(1,len(keys)):
                    this_position[key[i]] = position_line[i]
                self.motion_queue.append("duration" : position_line[0], "position" : this_position)
             
if __name__ == '__main__':
    baxter = BaxterInterface()
    baxter.interact()
    

