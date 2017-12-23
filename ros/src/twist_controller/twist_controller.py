import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter
import numpy as np

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Tuning parameters for throttle/brake PID controller
PID_VEL_P = 0.9
PID_VEL_I = 0.0005
PID_VEL_D = 0.07



class Controller(object):
    def __init__(self, *args, **kwargs):
        rospy.loginfo('TwistController: Start init')

        ######Reading input parameters#################
        self.sampling_rate = kwargs["sampling_rate"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]

        # brake_deadband is the interval in which the brake would be ignored
        # the car would just be allowed to slow by itself/coast to a slower speed
        self.brake_deadband = kwargs["brake_deadband"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]

        # parameters to use for the Yaw (steering) controller
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]
        self.max_throttle_percentage = kwargs["max_throttle_percentage"]
        self.max_braking_percentage = kwargs["max_braking_percentage"]

        self.delta_t = 1/self.sampling_rate
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity \
            * GAS_DENSITY) * self.wheel_radius

        self.past_vel_linear = 0.0
        self.current_accel = 0.0

        self.lowpass = LowPassFilter(self.accel_limit, self.delta_t)
        self.pid_vel_linear = PID(2.0, 0.4, 0.1, mn=self.max_braking_percentage, mx=self.max_throttle_percentage)

        #self.low_pass_filter_accel = LowPassFilter(LPF_ACCEL_TAU, self.delta_t)



        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        #self.pid_vel_linear = PID(PID_VEL_P, PID_VEL_I, PID_VEL_D,
        #                          self.decel_limit, self.accel_limit)

        # second controller to get throttle signal between 0 and 1
        #self.accel_pid = PID(PID_ACC_P, PID_ACC_I, PID_ACC_D, 0.0, 0.75)


        # Initialise Yaw controller - this gives steering values using
        # vehicle attributes/bicycle model
        # Need to have some minimum speed before steering is applied
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=5.0,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)

        rospy.loginfo('TwistController: Complete init')
        rospy.loginfo('TwistController: Steer ratio = ' + str(self.steer_ratio))

    def control(self, required_vel_linear,required_vel_angular,
                current_vel_linear):


        self.ideal_linear_velocity = required_vel_linear
        self.ideal_angular_velocity = required_vel_angular
        self.current_linear_velocity = current_vel_linear

        steer = self.ideal_angular_velocity * self.steer_ratio
        brake = 0.
        throttle = 0.

        # convert current velocity to ideal velocity delta percentage
        if abs(self.ideal_linear_velocity) > abs(self.current_linear_velocity):
            if self.ideal_linear_velocity < 0.:
                throttle = -0.01
            else:
                factor = self.ideal_linear_velocity
                throttle = np.max([np.min(
                    [4 * (self.ideal_linear_velocity - self.current_linear_velocity + 0.1) / factor,
                     self.max_throttle_percentage]), self.max_braking_percentage])

        elif self.current_linear_velocity > 0.1:
            factor = self.current_linear_velocity
            throttle = np.max([np.min([4 * (self.ideal_linear_velocity - self.current_linear_velocity - 0.1) / factor,
                                       self.max_throttle_percentage]), self.max_braking_percentage])
        else:
            throttle = -0.01
        if throttle < 0.:
            brake = -throttle
            throttle = 0.

        rospy.loginfo('TwistController.control: brake, current velocity, throttle: '+str(brake)+ \
                      ' , '+str(self.current_linear_velocity)+' , '+str(throttle))

        # steering - yaw controller takes desired linear, desired angular, current linear as params
        #steering = required_vel_angular * self.steer_ratio
        steering = self.yaw_controller.get_steering(required_vel_linear,
                                                    required_vel_angular,
                                                    current_vel_linear)

        # uncomment for debugging
        #if throttle <> 0.0:
        #   rospy.loginfo('TwistController: Accelerating = ' + str(throttle))
        #if brake <> 0.0:
        #    rospy.loginfo('TwistController: Braking = ' + str(brake))
        if abs(steering) <> 0.0:
            #rospy.loginfo('TwistController: Steering = ' + str(steering))
            rospy.loginfo('Veer: Steering = ' + str(steering) + ', required = ' + str(required_vel_angular))

        return throttle, brake, steering

    def reset(self):
        self.pid_vel_linear.reset()