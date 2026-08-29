#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

rospy.init_node('decisao')
rate = rospy.Rate(10)

def callback(msg):
	x = msg.x
	y = msg.y
	theta = msg.theta
	
	if(x>0):
		vel = Twist()
		vel.linear.x = 2.0
		vel.angular.z = 1.0
		pub.publish(vel)
		rate.sleep()
		

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
sub = rospy.Subscriber('/turtle1/pose', Pose, callback)
rospy.spin()



	
	
