import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter
from simplelowpass import SimpleLowPassFilter
import numpy as np

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        vehicle_mass    = kwargs['vehicle_mass']
        fuel_capacity   = kwargs['fuel_capacity']
        self.brake_deadband  = kwargs['brake_deadband']
        self.decel_limit 	= kwargs['decel_limit']
        accel_limit 	= kwargs['accel_limit']
        wheel_radius 	= kwargs['wheel_radius']
        wheel_base 		= kwargs['wheel_base']
        steer_ratio 	= kwargs['steer_ratio']
        max_lat_accel 	= kwargs['max_lat_accel']
        max_steer_angle = kwargs['max_steer_angle']
        sampling_rate   = kwargs['sampling_rate']
        self.max_braking_percentage= kwargs['max_braking_percentage']
        self.max_throttle_percentage = kwargs['max_throttle_percentage']
        min_speed = 5.0

        self.brake_tourque_const = (vehicle_mass + fuel_capacity * GAS_DENSITY) * wheel_radius
        self.current_dbw_enabled = False

        yaw_params = [wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle]
        self.yaw_controller = YawController(*yaw_params)

        self.linear_pid = PID(1.0, 0.0005, 0.07, self.decel_limit, accel_limit)

        self.tau_correction = 0.2
        self.ts_correction = 1.0/sampling_rate
        self.low_pass_filter_correction = LowPassFilter(self.tau_correction, self.ts_correction)

        self.previous_time = None

        rospy.loginfo('TwistController: Complete init')

    def update_sample_step(self):
        current_time = rospy.get_time()
        sample_step = current_time - self.previous_time if self.previous_time else 0.05
        self.previous_time = current_time
        return sample_step

    def control(self,
                linear_velocity_setpoint, angular_velocity_setpoint,
                linear_current_velocity, angular_current,
                dbw_enabled,current_location):

        if (not self.current_dbw_enabled) and dbw_enabled:
            self.current_dbw_enabled = True
            self.linear_pid.reset()
            self.previous_time = None
        else:
            self.current_dbw_enabled = False

        linear_velocity_error = linear_velocity_setpoint - linear_current_velocity

        sample_step = self.update_sample_step()

        velocity_correction = self.linear_pid.step(linear_velocity_error, sample_step)
        velocity_correction = self.low_pass_filter_correction.filt(velocity_correction)


        if abs(linear_velocity_setpoint) < 0.01 and abs(linear_current_velocity) < 0.1:
            velocity_correction = self.decel_limit

        throttle = velocity_correction
        brake = 0.

        if throttle < 0.:
            decel = abs(throttle)
            # [alexm]NOTE: let engine decelerate the car if required deceleration below brake_deadband
            brake = self.brake_tourque_const * decel if decel > self.brake_deadband else 0.
            throttle = 0.

        #tempoary elif as throttle reach almost to .90
        elif throttle > 0.4:
            throttle = 0.4


        steering = self.yaw_controller.get_steering(linear_velocity_setpoint, angular_velocity_setpoint,
                                                               linear_current_velocity)



        rospy.logwarn('####Controller.control: velocity_correction,throttle, brake, steering, sample_step '+str(velocity_correction)+' , '+str(throttle)+' , '+str(brake) +' , '+ str(steering)+' , '+str(sample_step))

        return throttle, brake, steering


    def level5_control(self, linear_velocity_target, angular_velocity_target, linear_velocity_current):

        # Throttle xor brake
        brake = 0.
        throttle_correction = self.velocity_pid.step(linear_velocity_target - linear_velocity_current, 0.02)
        if throttle_correction > 0.:
            throttle = throttle_correction
            throttle = self.lowpassFilt.filt(throttle)
        elif throttle_correction < -self.brake_deadband:
            throttle = 0.
            brake = -throttle_correction
        else:
            throttle = 0.
        # hold brake while stopped at red light, until light changes
        if (linear_velocity_target <= 0.01) and (brake < self.brake_deadband):
            brake = self.brake_deadband

        # Steering
        # Use yawController for simulator
        if self.brake_deadband > 0.1:
            if linear_velocity_target > self.topVelocity:  # mitigate rapid turning
                self.topVelocity = linear_velocity_target
            if linear_velocity_current > 0.05:
                steering = self.yawController.get_steering(self.topVelocity, angular_velocity_target,
                                                           linear_velocity_current)
            else:
                steering = 0.
        # ...and alternate approach (to match reference) on the test site:
        else:
            steering = angular_velocity_target * self.steer_ratio
        return throttle, brake, steering

    def chen_control(self, required_vel_linear,required_vel_angular,
                current_vel_linear):


        self.ideal_linear_velocity = required_vel_linear
        self.ideal_angular_velocity = required_vel_angular
        self.current_linear_velocity = current_vel_linear

        steering = self.ideal_angular_velocity * self.steer_ratio
        brake = 0.
        throttle = 0.05

        # convert current velocity to ideal velocity delta percentage
        #if abs(self.ideal_linear_velocity) > abs(self.current_linear_velocity):
        #    if self.ideal_linear_velocity < 0.:
        #        throttle = -0.01
        #    else:
        #        factor = self.ideal_linear_velocity
        #        throttle = np.max([np.min(
        #            [4*(self.ideal_linear_velocity - self.current_linear_velocity + 0.1) / factor,
        #             self.max_throttle_percentage]), self.max_braking_percentage])

        #elif self.current_linear_velocity > 0.1:
        #    factor = self.current_linear_velocity
        #    throttle = np.max([4*np.min([ (self.ideal_linear_velocity - self.current_linear_velocity - 0.1) / factor,
        #                               self.max_throttle_percentage]), self.max_braking_percentage])
        #else:
        #    throttle = -0.01
        #if throttle < 0.:
        #    brake = -throttle
        #    throttle = 0.


        rospy.loginfo('TwistController.control: brake, current velocity, throttle: '+str(brake)+ \
                      ' , '+str(self.current_linear_velocity)+' , '+str(throttle))

        # steering - yaw controller takes desired linear, desired angular, current linear as params
        steering = self.yaw_controller.get_steering(required_vel_linear,
                                                    required_vel_angular,
                                                    current_vel_linear)

        #steering = self.lowpass.filt(steering)

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