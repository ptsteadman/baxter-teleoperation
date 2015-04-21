'''
Created on Mar 30, 2015

@author: Alice
'''
import threading
class StopThread(threading.Thread):
    '''
    classdocs
    '''


    def __init__(self, some_lambda_function):
        super(StopThread, self).__init__()
        self._stop = threading.Event()
        self.job = some_lambda_function
        
    def stop(self):
        self._stop.set()

    def stopped(self):
        return self._stop.isSet()
    
    
    def run(self):
        while not self._stop.isSet():
            self.job()
        print "thread executed"