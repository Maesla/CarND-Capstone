from pid import PID


class Speed_Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        
        #self.pid = PID(0.0476519, -8.561e-05, 0.201947)
        self.pid = PID(1.0, 0.0, 0.0)


    def control(self, cte_speed):
        throttle = self.pid.step(cte_speed, 1.0/50) #TODO remove 1/50 hardcode
        throttle = self.clamp(throttle, 0.0,1.0)
        return throttle, 0.
        
    def clamp(self, value, min_value, max_value):
		if value > max_value:
			return max_value
		elif value < min_value:
			return min_value
		return value
		
    def reset(self):
		self.pid.reset()
