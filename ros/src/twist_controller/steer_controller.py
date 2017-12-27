from pid import PID


class Steer_Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.pid = PID(1.0,0,0)
        pass


    def control(self, cte_yaw):
        steer = self.pid.step(cte_yaw, 1.0/50) #TODO remove 1/50 hardcode
        steer = self.clamp(steer, -1.0, 1.0) # TODO, not sure if the range is -1,1
        return steer
        
    def clamp(self, value, min_value, max_value):
		if value > max_value:
			return max_value
		elif value < min_value:
			return min_value
		return value
    def reset(self):
		self.pid.reset()
