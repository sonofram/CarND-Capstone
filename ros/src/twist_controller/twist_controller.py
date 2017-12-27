import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter
from simplelowpass import SimpleLowPassFilter
import numpy as np

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_V =44.704

# Tuning parameters for throttle/brake PID controller
#PID_VEL_P = 0.9
#PID_VEL_I = 0.0005
#PID_VEL_D = 0.07

PID_VEL_P = 0.9
PID_VEL_I = 0.05
PID_VEL_D = 0.3

#PID_ACC_P = 0.4
#PID_ACC_I = 0.05
#PID_ACC_D = 0.0

#PID_ACC_P = 0.8
#PID_ACC_I = 0.05
#PID_ACC_D = 0.50

PID_STR_P = 0.9
PID_STR_I = 0.05
PID_STR_D = 0.9


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass    = kwargs['vehicle_mass']
        self.fuel_capacity   = kwargs['fuel_capacity']
        self.brake_deadband  = kwargs['brake_deadband']
        self.decel_limit 	= kwargs['decel_limit']
        self.accel_limit 	= kwargs['accel_limit']
        self.wheel_radius 	= kwargs['wheel_radius']
        self.wheel_base 		= kwargs['wheel_base']
        self.steer_ratio 	= kwargs['steer_ratio']
        self.max_lat_accel 	= kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.sampling_rate   = kwargs['sampling_rate']
        self.max_braking_percentage= kwargs['max_braking_percentage']
        self.max_throttle_percentage = kwargs['max_throttle_percentage']
        min_speed = 5.0

        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        #Variable used in control function
        self.delta_t = 1.0 / self.sampling_rate
        self.current_accel = 0.0
        self.linear_past_velocity = 0.0

        #yaw controller
        yaw_params = [self.wheel_base, self.steer_ratio, min_speed,
                      self.max_lat_accel,
                      self.max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)

        #Velocity PID
        self.pid_vel_linear = PID(PID_VEL_P, PID_VEL_I, PID_VEL_D,
                                  self.decel_limit, self.accel_limit)


        self.steer_pid = PID(PID_STR_P, PID_STR_I, PID_STR_D, -self.max_steer_angle/2, self.max_steer_angle/2)

        #self.tau_steer_correction = 0.5
        #self.ts_steer_correction = self.delta_t
        #self.low_pass_filter_steer = LowPassFilter(self.tau_steer_correction, self.ts_steer_correction)

        self.last_time = None
        self.last_steering = 0.0

        rospy.loginfo('TwistController: Complete init')


    def control(self,
                linear_velocity_setpoint,
                angular_velocity_setpoint,
                linear_current_velocity,
                steering_feedback):

        throttle, brake, steering = 0.0, 0.0, 0.0

        time = rospy.get_time()


        if self.last_time is not None:

            dt = time - self.last_time

            self.last_time = time

            # update
            if linear_current_velocity < 1.0:
                self.steer_pid.reset()
                self.pid_vel_linear.reset()

            # use velocity controller compute desired accelaration
            linear_velocity_error =  (linear_velocity_setpoint - linear_current_velocity)/MAX_V
            desired_accel = self.pid_vel_linear.step(linear_velocity_error, dt) #self.delta_t)



            if desired_accel > 0.0:
                #if desired_accel < self.accel_limit:
                #    throttle = self.accel_limit
                #else:
                if desired_accel > 0.2:
                    throttle = 0.2
                else:
                    throttle = desired_accel
                brake = 0.0
            else:
                throttle = 0.0
                # reset just to be sure
                #self.accel_pid.reset()
                if abs(desired_accel) > self.brake_deadband:
                    # don't bother braking unless over the deadband level
                    # make sure we do not brake to hard
                    if abs(desired_accel) > abs(self.decel_limit):
                        brake = abs(self.decel_limit) * self.brake_torque_const
                    else:
                        brake = abs(desired_accel) * self.brake_torque_const

            steering = self.yaw_controller.get_steering(linear_velocity_setpoint, angular_velocity_setpoint,
                                                                   linear_current_velocity)

            steering = self.steer_pid.step(steering-steering_feedback, dt)

            return throttle, brake, steering
        else:
            self.last_time = time
            return 0.0,0.0,0.0

    def reset(self):
        self.pid_vel_linear.reset()