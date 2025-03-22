from threading import Lock

class RealtimeBuffer:
    '''
    This class implements a real-time buffer for a single object.
    '''
    def __init__(self):
        self.rt_obj = None
        self.non_rt_obj = None
        self.new_data_available = False
        self.lock = Lock()
        
    def writeFromNonRT(self, obj):
        '''
        Write data to non-realtime object. If a real-time thread 
        is reading the non-realtime object, wait until it finish.
        
        Parameters:
            obj: object to be written
        '''
        self.lock.acquire(blocking=True)
        self.non_rt_obj = obj
        self.new_data_available = True
        self.lock.release()
        
    def readFromRT(self):
        '''
        if no thread is writing and new data is available, update rt-object 
        with non-rt object.
        
        Returns:
            rt_obj: real-time object
        '''
        # try to lock
        if self.lock.acquire(blocking=False):
            if self.new_data_available:
                temp = self.rt_obj
                self.rt_obj = self.non_rt_obj
                self.non_rt_obj = temp
                self.new_data_available = False
            self.lock.release()
        return self.rt_obj
    
    def reset(self):
        '''
        Reset the buffer to None
        '''
        self.writeFromNonRT(None)
