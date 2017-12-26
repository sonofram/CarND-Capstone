#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import os
import csv
from twist_controller import Controller
from styx_msgs.msg import Lane, Waypoint


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

class BaseNode(object):
    def __init__(self):
        rospy.init_node('base_node', log_level=rospy.INFO)
        rospy.logout("base_node initiated...........")

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.loginfo("base_node: Subscriber initiated")

        self.base_waypoints_list = []
        self.base_waypoints_data = []

        if DUMP is True:
            base_path = os.path.dirname(os.path.abspath(__file__))
            self.base_waypoints_file = os.path.join(base_path, 'base_waypoints.csv')


        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if len(self.base_waypoints_list) == 0:
                continue;

            for i in range(0, len(self.base_waypoints_list) - 1):
                rospy.logwarn('dx,dy: '+str(self.base_waypoints_list[i].pose.pose.position.x)+' , '+str(self.base_waypoints_list[i].pose.pose.position.y))
                self.base_waypoints_data.append({
                    'dx': self.base_waypoints_list[i].pose.pose.position.x,
                    'dy': self.base_waypoints_list[i].pose.pose.position.y})

            rospy.loginfo('BaseNode.loop base_waypoints_data array length: ' + str(len(self.base_waypoints_data)))

            self.base_waypoints_list = []

            rate.sleep

        fieldnames2 = [
            'dx','dy'
        ]


        with open(self.base_waypoints_file, 'w') as csv2file:
            writer2 = csv.DictWriter(csv2file, fieldnames=fieldnames2)
            writer2.writeheader()
            writer2.writerows(self.base_waypoints_data)

    def waypoints_cb(self, waypoints):
        self.base_waypoints_list = waypoints.waypoints
        #rospy.loginfo('base_node.waypionts_cb: length = ' + str(len(self.base_waypoints_list)))
        rospy.loginfo('BaseNode.waypionts_cb: length = ' + str(len(self.base_waypoints_list)))


if __name__ == '__main__':
    BaseNode()