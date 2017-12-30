from speed_controller import Speed_Controller
from steer_controller import Steer_Controller


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.speed_controller = Speed_Controller()
        self.steer_controller = Steer_Controller()

    def setup(self, rate, vehicle_mass, fuel_capacity, wheel_radius,
              max_deceleration, wheel_base, steer_ratio, max_lat_accel,
              min_speed, max_steer_angle):
        self.speed_controller.setup(rate, vehicle_mass, fuel_capacity, GAS_DENSITY, wheel_radius, max_deceleration)
        self.steer_controller.setup(wheel_base, steer_ratio, max_lat_accel,
                                    min_speed, max_steer_angle)

    def control(self, target_speed, target_yaw, current_speed):
        throttle, brake = self.speed_controller.control(current_speed, target_speed)
        steer = self.steer_controller.control(target_speed,
                                              target_yaw,
                                              current_speed)
        return throttle, brake, steer

    def reset(self):
        self.speed_controller.reset()
