import code
import time
import multiprocessing


banner = "WELCOME TO BAXTER"
import rospy

import baxter_interface
class BaxterInterfaceTeleop(object):

    def __init__(self, baxter):
        self.test = "test"
        self.motion_state = "idle"
        self.__head = baxter_interface.Head()
        
        self.loop = None
        self.queue = multiprocessing.Queue()
        self.current_image = None
        self.stopped = False
        self.loop = multiprocessing.Process(target=self.control_loop,
                args=(10,))
        self.loop.start()
        self.baxter = baxter
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
        self.show_image('on.png')
        if transition > 1:
            if transition == 2:
                time.sleep(3)
                self.show_image('eyesClosed.png')
                time.sleep(1)
                self.show_image('eyesOpened.png')
                time.sleep(.25)
                self.show_image('eyesClosed.png')
                time.sleep(.25)
                self.show_image('eyesOpened.png')
                time.sleep(.5)
                self.show_image('eyesClosed.png')
                time.sleep(.15)
                self.show_image('eyesOpened.png')
                time.sleep(.5)
                self.show_image("smile.png")
                time.sleep(3)
                self.show_image("eyesOpened.png")
            elif transition == 3:
                time.sleep(3)
                self.show_image("on with patrick.png")
    def stop_teleoperation(self):
        self.motion_state = "idle"

    
    def show_image(self, new_image):
        if not self.current_image == new_image:
            send_image(new_image)
            self.current_image = new_image
            
            
        
    def nod(self):
        print "baxter nods"
        self.__head.command_nod()
        
    def pan(self, degree):
        self.__head.set_pan(degree)
        
    def interact(self):
        baxter = self
        code.interact(banner=banner, local=locals())

if __name__ == '__main__':
    baxter = BaxterInterface()
    baxter.interact()
    
def send_image(path):
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

