import code
import time
import multiprocessing


banner = "WELCOME TO BAXTER"

class BaxterInterface(object):

    def __init__(self):
        self.test = "test"
        self.motion_state = "idle"
        self.loop = None
        self.queue = multiprocessing.Queue()
        self.current_image = None
        self.stopped = False
        self.loop = multiprocessing.Process(target=self.control_loop,
                args=(10,))
        self.loop.start()
        
    def control_loop(self, t):
        while not self.stopped:
            if not self.queue.empty():
                self.motion_state = self.queue.get()
            print "\n" +  self.motion_state
            time.sleep(t)
        
    def idle(self): 
        self.queue.put("idle")
    
    def standby(self): 
        self.queue.put("standby")
        
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

