#!/usr/bin/env python
import rospy
import pandas as pd
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import numpy as np


def poseCallback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion(
        [rot_q.x, rot_q.y, rot_q.z, rot_q.w])


data = pd.read_csv('../results/data.csv')
Y = data['x'].values
X = data['y'].values

x, y, theta = 0.0, 0.0, 0.0

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odom", Odometry, poseCallback)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()

rate = rospy.Rate(100)
i = 0
goal = Point()
goal_reached = False
while not goal_reached:
    if i < len(X):
        if (i + 5) > len(X):
            goal.x = -10.0 + (X[-1] / 5.0)
            goal.y = 10.0 - (Y[-1] / 5.0)
        else:
            goal.x = -10.0 + (X[i] / 5.0)
            goal.y = 10.0 - (Y[i] / 5.0)
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
        pub.publish(speed)
        goal_reached = True
        print("Goal reached!!")
        break

    inc_x = goal.x - x
    inc_y = goal.y - y
    rospy.loginfo("Moving to: x=%f ,y=%f", goal.x, goal.y)
    angle_to_goal = atan2(inc_y, inc_x)
    distance_to_goal = np.sqrt(inc_x * inc_x + inc_y * inc_y)

    xy_tol, th_tol = 0.1, 0.1
    if distance_to_goal >= xy_tol: 
        if angle_to_goal - theta > th_tol:
            speed.linear.x = 0.0
            speed.angular.z = 1
        elif angle_to_goal - theta < -th_tol:
            speed.linear.x = 0.0
            speed.angular.z = -1
        else:
            speed.linear.x = 1.5
            speed.angular.z = 0.0
        pub.publish(speed)

    else:
        i += 4

    rate.sleep()
