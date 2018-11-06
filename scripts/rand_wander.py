#!/usr/bin/env python

"""
Authors: Garrett Folks, Jason Nolasco, Zamua Nasrawt

"""

import rospy
import math
import random

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan 

class Wander(object):
    def __init__(self):
        rospy.init_node('Wander')

        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)

        self.thrust = Twist()
        self.rate = rospy.Rate(10)
        self.bumper = BumperEvent()
        self.scan = LaserScan()

    def scan_callback(self, scan_msg):
        self.scan = min(scan_msg.ranges)

    def bumper_callback(self, bumper_msg):
        self.bumper = bumper_msg.state

    def run(self):

        while not rospy.is_shutdown():
            if self.bumper == 1 or self.scan <= 1 or self.scan == math.isnan:
                self.thrust.linear.x = 0
                self.thrust.angular.z = random.random()
            else:
                self.thrust.linear.x = .2
                self.thrust.angular.z = 0

            self.twist_pub.publish(self.thrust)
            self.rate.sleep()

if __name__ == "__main__":
    node = Wander()
    node.run()
