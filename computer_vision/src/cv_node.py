#! /usr/bin/env python
import rospy
from computer_vision.msg import target

while True:
	rospy.init_node('CV')
	cv_pub = rospy.Publisher('target', target, queue_size=10)
	rate = rospy.Rate(10)
	final_message = target()
	while not rospy.is_shutdown():
		final_message.buoy1 = True
                final_message.buoy1x = 4.1
		final_message.buoy1y = 0.1
		final_message.buoy1_distance = 14
		cv_pub.publish(final_message)
		rate.sleep()

	else:
		exit()
