import time
import cutils

class Controller2D(object):
    def __init__(self):
        self.vars  = cutils.CUtils()
        self.currenttime = self.current_time_in_nanoseconds()
        self.vars.create_var('t_prev', 0.0)

    def current_time_in_nanoseconds(self):
        # Get current epoch time in seconds
        epoch_time_seconds = time.time()

        # Convert seconds to nanoseconds
        epoch_time_nanoseconds = int(epoch_time_seconds * 1e9)

        return epoch_time_nanoseconds

# Example usage
timep = Controller2D()
timep.vars.t_prev  = timep.currenttime
while(True):
    epoch_time_ns = timep.current_time_in_nanoseconds()
    print(f"Current epoch time in nanoseconds: {epoch_time_ns}")
    diff = (epoch_time_ns-timep.vars.t_prev)/(1e9)
    print("previous time: ",timep.vars.t_prev)
    timep.vars.t_prev       = epoch_time_ns
    
    
    
    print("difference: ",diff)




