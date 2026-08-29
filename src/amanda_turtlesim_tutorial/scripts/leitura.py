#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def callback(msg):
	print("X: ", msg.x)
	print("Y: ", msg.y)
	print("Theta: ", msg.theta)
	
def main():
	rospy.init_node('leitor')
	sub = rospy.Subscriber('/turtle1/pose', Pose, callback)
	rospy.spin()
	
if __name__ == '__main__':
    main()
	


