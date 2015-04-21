#!/usr/bin/env python

import code
from collections import deque
import time
import multiprocessing
import rospy
import tf2_ros
import baxter_interface
from baxter_interface import CHECK_VERSION

banner = "WELCOME TO BAXTER"

class BaxterInterface(object):

    def __init__(self):
        self.state = dict()
        self.state["mode"] = "idle"
        self.image_timer = 0
        self.image_queue = deque()
        self.motion_queue = deque()
        self.motion_timer = 0
        self.queue = multiprocessing.Queue()
        self.loop = multiprocessing.Process(target=self.control_loop,
                args=(10,))
        self.loop.start()
        
    def set_rate(self, t):
        """ Sets the rate to send commands to the robot """
        self.rate = rate


    def control_loop(self, t):
        while not rospy.is_shutdown():
            # empty the queue of user console commands
            while not self.queue.empty():
                command = self.queue.get()
                self.state[command['k']] = command['v']

            # set baxter's joint angles based on current mode
            if self.state["mode"] == "idle":
                left.set_joint_positions(IDLE_ANGLES['left'])
                right.set_joint_positions(IDLE_ANGLES['right'])
            if self.state["mode"] == "file_playback":
                # set joint positions according to csv file/timer

            if self.state["mode"] == "teleoperated":
                joint_angles = get_joint_angles(user, tfBuffer, test, mirrored)
                if joint_angles is not None:
                    left.set_joint_positions(joint_angles['left'])
                    right.set_joint_positions(joint_angles['right'])

            # set screen image
            if len(self.image_queue) > 0:
                if self.image_timer > self.image_queue[0]['time']:
                    self.image_queue.pop()

            rate.sleep()
        print "Rospy shutdown, exiting loop."
        
    def idle(self): 
        self.queue.put({"mode":"idle"})
    
    def standby(self): 
        self.play_file("standby.csv")
        self.play_image("on.png", 5)
        
    def start_teleoperation(self, transition):
        # first, execute transition
        self.motion_state = "teleoperation"
        
    def stop_teleoperation(self):
        self.motion_state = "idle"

    def play_file(self, filename):
        ''' Play a csv file of joint angles '''
    
    def show_image(self, new_image):
        self.current_image = new_image

    def nod(self):
        print "baxter nods"

    def interact(self):
        baxter = self
        code.interact(banner=banner, local=locals())

if __name__ == '__main__':
    baxter = BaxterInterface()
    baxter.interact()

