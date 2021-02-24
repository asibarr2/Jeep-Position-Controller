#!/usr/bin/env python

import rospy
import math
import numpy as np
from pid import PID
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import subprocess

class Controller():
    def __init__(self, initial_position, initial_orientation, odometry_topic, velocity_topic, k):
        self.x = initial_position[0]
        self.y = initial_position[1]
        self.z = initial_position[2]
        self.roll = initial_orientation[0]
        self.pitch = initial_orientation[1]
        self.yaw = initial_orientation[2]
        self.odom = odometry_topic
        self.cmd_vel = velocity_topic
        self.distance = 0.0
        self.heading = 0.0
        self.pid = PID(k)
        self.last_time = 0.0
        self.threshold = 0.1

    def odomCallback(self, data):
        self.x = float(round(data.pose.pose.position.x, 1))
        self.y = float(round(data.pose.pose.position.y, 1))
        self.z = float(round(data.pose.pose.position.z, 1))
        orientation_quaternion = data.pose.pose.orientation
        orientation_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)

    ####
    #    Parameters: target = [x*, y*]
    #    Returns: Euclidean Distance Error, Heading Error
    ####
    def calculateError(self, target):
        delta_x = np.clip(target[0] - self.x, -1e50, 1e50)
        delta_y = np.clip(target[1] - self.y, -1e50, 1e50)
        desired_heading = math.atan2(delta_y, delta_x)

        self.heading = desired_heading - self.yaw
        delta_x2 = delta_x**2
        delta_y2 = delta_y**2
        if math.isinf(delta_x2):
            delta_x2 = 1e25
        if math.isinf(delta_y2):
            delta_y2 = 1e25
        self.distance = math.sqrt(delta_x2 + delta_y2)

    def moveToTarget(self):

        rospy.init_node('mouseToJoy', anonymous = True)
        #### Setup Odometry Subscriber
        rospy.Subscriber(self.odom, Odometry, self.odomCallback, queue_size=1, tcp_nodelay=True)

    	#### Setup Velocity Publisher
        velocityPublisher = rospy.Publisher(self.cmd_vel, Twist, queue_size=1, tcp_nodelay=True)
        rate = rospy.Rate(20) # 20hz
        data = Twist()

	x_target = float(input("Enter X Target: \n"))
	y_target = float(input("Enter Y Target: \n"))
        target = np.array([x_target, y_target])

        while not rospy.is_shutdown():
            self.calculateError(target)
            dt = rospy.Time.now().to_sec() - self.last_time
            self.pid.calculatePID(self.distance, self.heading, dt)
            self.last_time = rospy.Time.now().to_sec()

            print(self.x, self.y)

            if math.fabs(self.distance) > self.threshold:
                data.linear.x = 0.2
                #data.linear.x = self.pid.velocity
                # msg.angular.z = 100
                data.angular.z = self.pid.steering

            else:
                data.linear.x = 0.0
                data.angular.z = 0.0
                print("Target Reached")
		subprocess.call("rosservice call /zed/zed_node/set_pose 0 0 0 0 0 0 ")
		velocityPublisher.publish(data)
		rate.sleep()
		sys.exit(1)

	    velocityPublisher.publish(data)
	    rate.sleep()

if __name__ == '__main__':
    init_position = np.array([0.0, 0.0, 0.0])
    init_orientation = np.array([0.0, 0.0, 0.0])
    odomTopic = "zed/zed_node/odom"
    velTopic = "/cmd_vel_sub"
    # PID Gain Parameters: index 0-2 = velocity, index 3-5 = steering
    k = [5.0, 0.0, 0.0, 10.0, 0.0, 0.0]

    jeep = Controller(init_position, init_orientation, odomTopic, velTopic, k)
    jeep.moveToTarget()
