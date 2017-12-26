from pid import PID


class Speed_Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid = PID(1.0,0,0)
        pass


    def control(self, cte_speed):
        throttle = self.pid.step(cte_speed, 1.0/50) #TODO remove 1/50 hardcode
        
        return throttle, 0.
