#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import os
import csv
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.
You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.
One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.
We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.
Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.
'''

DUMP = True #True

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.INFO)
        rospy.logout("dbw_node initiated...........")

        self.required_vel_linear = None
        self.required_vel_angular = None
        self.current_vel_linear = None

        self.last_required_vel_angular = 0.0
        self.count_required_vel_angular = 0

        # default to drive-by-wire not enabled - will pick this up from the topic...
        self.dbw_enabled = False
        self.sampling_rate = 50.0 # 50Hz

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_throttle_percentage = rospy.get_param('~max_throttle_percentage', 0.1)
        max_braking_percentage = rospy.get_param('~max_braking_percentage', -0.1)

        #################PUBLISH################################
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)


        # TODO: Create `TwistController` object
        self.controller = Controller(sampling_rate=self.sampling_rate,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     brake_deadband=brake_deadband,
                                     vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     wheel_radius= wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle,
                                     min_velocity=0.0,
                                     max_throttle_percentage=max_throttle_percentage,
                                     max_braking_percentage=max_braking_percentage)

        ##################SUBSCRIBE##############################
        #Check when manual control taken over.
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # /current_velocity topic gives the current velocity of the vehicle
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)
        # /twist_cmd topic is the output from the vehicles waypoint controller
        # As implemented in waypoint_follower / pure_pursuit given code
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)

        rospy.loginfo("dbw_node: Publisher and Subscriber initiated")

        self.dbw_data = []

        if DUMP is True:
            base_path = os.path.dirname(os.path.abspath(__file__))
            self.dbwfile = os.path.join(base_path, 'dbw_node.csv')

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.sampling_rate) # Was 50Hz
        while not rospy.is_shutdown():
            if self.current_vel_linear is None or self.required_vel_linear is None \
            or not self.dbw_enabled:
                continue

            throttle, brake, steer = self.controller.control(
                self.required_vel_linear,
                self.required_vel_angular,
                self.current_vel_linear)

            if DUMP is True:
                self.dbw_data.append({
                    'dbw_enabled': self.dbw_enabled,
                    'current_vel_linear': self.current_vel_linear,
                    'required_vel_linear': self.required_vel_linear,
                    'required_vel_angular': self.required_vel_angular,
                    'throttle': throttle,
                    'brake': brake,
                    'steer': steer})

            rospy.loginfo("dbw_node.loop: throttle, brake, streer"+str(throttle)+' , '+str(brake)+' , '+str(steer))
            self.publish(throttle, brake, steer)
            rate.sleep()


        rospy.loginfo('dbw_node.loop dwb_data array length: '+str(len(self.dbw_data)))
        if DUMP is True:
            fieldnames = ['dbw_enabled',
                          'current_vel_linear',
                          'required_vel_linear',
                          'required_vel_angular',
                          'throttle',
                          'brake',
                          'steer']

            with open(self.dbwfile, 'w') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.dbw_data)


    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        #rospy.loginfo('TwistController: Steering = ' + str(steer))
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_enabled_cb(self, msg):
        # check if drive-by-wire is enabled (i.e. the car is not in manual mode)
        self.dbw_enabled = msg.data
        rospy.loginfo('dbw_node.dbw_enabled_cb: dbw_enabled = ' + str(self.dbw_enabled))

        if not self.dbw_enabled:
            self.controller.reset()

    def current_velocity_cb(self, msg):
        # store the current velocity TwistStamped message
        self.current_vel_linear = msg.twist.linear.x
        rospy.loginfo('dbw_node.current_velocity_cb: current_vel_linear = ' + str(self.current_vel_linear))

    def twist_cmd_cb(self, msg):
        # store the received TwistStamped message from the waypoint follower node
        self.required_vel_linear = msg.twist.linear.x
        self.required_vel_angular = msg.twist.angular.z
        rospy.loginfo('dbw_node.twist_cmd_cb: required_vel_linear = ' + str(self.required_vel_linear))
        rospy.loginfo('dbw_node.twist_cmd_cb: required_vel_angular = ' + str(self.required_vel_angular))

        # debugging veering issue
        if self.required_vel_angular <> self.last_required_vel_angular:
            rospy.loginfo('Veer: received new angular velocity required : ' + str(self.required_vel_angular) + ', count = ' + str(self.count_required_vel_angular))
            self.count_required_vel_angular = 0
        self.last_required_vel_angular = self.required_vel_angular
        self.count_required_vel_angular += 1

if __name__ == '__main__':
    DBWNode()