#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def publisher():
	rospy.init_node('quadrado')
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)
	
	vel = Twist()
	
	for i in range(4):
		vel.linear.x = 2.0
		vel.angular.z = 0.0
		
		for j in range(20):
			pub.publish(vel)
			rate.sleep()
			
		vel.linear.x = 0.0
		vel.angular.z = 0.0
		
		for k in range(5):
			pub.publish(vel)
			rate.sleep()
			
		vel.linear.x = 0.0
		vel.angular.z = 1.57
		
		for k in range(10):
			pub.publish(vel)
			rate.sleep()
			
	vel.linear.x = 0.0
	vel.angular.z = 0.0
	
	pub.publish(vel)

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
