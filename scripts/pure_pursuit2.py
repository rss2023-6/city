#!/usr/bin/env python

import rospy
import numpy as np
import time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32MultiArray
from sensor_msgs.msg import Joy
from visual_servoing.msg import ConeLocation

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic","/pf/pose/odom")
        self.speed            = 0.2 #filled in for testing purposes, please update
        self.wheelbase_length = 0.32 #flilled in for testing purposes, please update

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.drive_pub2 = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.line_sub = rospy.Subscriber("/line", Float32MultiArray, self.line_callback, queue_size = 20)

        self.brake = False # Boolean condition to determine whether to stop
        self.teleop_subscriber = rospy.Subscriber("/vesc/joy", Joy, self.tcb, queue_size=1)        
        self.pressed = False	
        self.radius = 0.4

    def tcb(self, msg):
        buttons = msg.buttons
        if(buttons[5] == 1):
            self.pressed = True
        else:
            self.pressed = False

    def line_callback(self, msg):
        x1 = msg.data[0]
        y1 = msg.data[1]
        x2 = msg.data[2]
        y2 = msg.data[3]
        carx1, cary1, carx2, cary2 = y1, x1, y2, x2
        goalx = (carx1*1.0 + carx2*1.0) / 2
        goaly = (cary1*1.0 + cary2*1.0) / 2
        self.drive_command(goalx, goaly)

    def drive_command(self, goalx, goaly):
        eta = np.arctan2(goaly, goalx)
        # R = self.lookahead / (2 * np.sin(eta))
        AckermannDrive = AckermannDriveStamped()
        AckermannDrive.header.stamp = rospy.Time.now()
        AckermannDrive.header.frame_id = "base_link"
        # if (self.brake):
        #     rospy.logerr("STOPPING)")
        #     AckermannDrive.drive.speed = 0
        #     AckermannDrive.drive.steering_angle = 0
        # else:
        AckermannDrive.drive.speed = self.speed
        angle = np.arctan2(2 * self.wheelbase_length * np.sin(eta), np.sqrt(goalx**2 + goaly**2))
        if abs(angle) >= 0.34:
            angle = 0.34 * np.sign(angle)
        AckermannDrive.drive.steering_angle = angle
        rospy.logerr(angle)

        #generalized sttering law by having a point ahead lecture slides
        # lfw = 0.05 #randomly defined based on lecture slides
        # AckermannDrive.drive.steering_angle = -1 * np.arctan(self.wheelbase_length * np.sin(eta) / (self.lookahead / 2  + lfw/np.cos(eta)))
        if (self.pressed):
            self.drive_pub2.publish(AckermannDrive)
        self.drive_pub.publish(AckermannDrive)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
