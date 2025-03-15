#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

rospy.init_node('odom_node')odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_link_state')
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

odom = Odometry()
header = Header()

header.frame_id = '/odom'
link = GetLinkStateRequest()
link.link_name = 'link_03'
link.reference_frame = 'world'

rate = rospy.Rate(10) # 10Hz
while not rospy.is_shutdown():
    try:
        result = get_link_srv(link)
        odom.pose.pose = result.link_state.pose
        odom.twist.twist = result.link_state.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        odom_pub.publish(odom)
        rate.sleep()
    except rospy.ROSTimeMovedBackwardsException:
        pass