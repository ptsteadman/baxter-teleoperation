import code
import teleoperate
import time
import threading


banner = "WELCOME TO BAXTER"

class BaxterInterface(object):

    def __init__(self):
        self.test = "test"
        self.motion_state = "idle"
        self.loop = None
        self.current_image = None
        self.stopped = False
        #self.loop = threading.Thread(target=self.control_loop(5))
        
    def control_loop(self, t):
        while not self.stopped:
            time.sleep(t)
        
    def idle(self): 
        self.motion_state = "idle"
    
    def standby(self): 
        self.motion_state = "standby"
        
    def start_teleoperation(self, transition):
        # first, execute transition
        self.motion_state = "teleoperation"
        
    def stop_teleoperation(self):
        self.motion_state = "idle"

    
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

