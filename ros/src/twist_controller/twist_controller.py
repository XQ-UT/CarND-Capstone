import rospy

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        
        # Yaw controller
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # PID controller
        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)
        
        # Low pass filter
        tau = 0.5 # cut-off frequency 
        ts = 0.02 # sample time
        self.vel_lpf = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        
        self.last_time = rospy.get_time()
        
    def control(self, proposed_linear_velocity, proposed_angular_velocity, curr_linear_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        curr_linear_velocity = self.vel_lpf.filt(curr_linear_velocity)
        steering = self.yaw_controller.get_steering(
            proposed_linear_velocity, 
            proposed_angular_velocity,
            curr_linear_velocity
        )
        
        velocity_error = proposed_linear_velocity - curr_linear_velocity
        self.last_velocity = curr_linear_velocity
        
        curr_time = rospy.get_time()
        sample_time = curr_time - self.last_time
        self.last_time = curr_time
        
        throttle = self.throttle_controller.step(velocity_error, sample_time)
        brake = 0
        
        if proposed_linear_velocity == 0.0 and curr_linear_velocity < 0.1:
            throttle = 0
            brake = 700
        elif throttle < 0.1 and velocity_error < 0:
            throttle = 0
            decel = max(velocity_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        
        return throttle, brake, steering
