from pid import PID


class Steer_Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid = PID(1.0,0,0)
        pass


    def control(self, cte_yaw):
        steer = self.pid.step(cte_yaw, 1.0/50) #TODO remove 1/50 hardcode
        
        return steer
