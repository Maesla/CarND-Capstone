from speed_controller import Speed_Controller
from steer_controller import Steer_Controller



GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.speed_controller = Speed_Controller()
        self.steer_controller = Steer_Controller()
        pass


    def control(self, cte_speed, cte_yaw):
        throttle, brake = self.speed_controller.control(cte_speed)
        steer = self.steer_controller.control(cte_yaw)
        return 0.5,0,0
        return throttle, brake, steer
