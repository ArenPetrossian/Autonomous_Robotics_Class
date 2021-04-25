#! /usr/bin/env python
import rospy
from state_machine.msg import task_desiredAction
from guidance_navigation_control.msg import controlCommand

def smach_data(smach_data):
	print (smach_data)

while True:
	rospy.init_node('GNC')
	gnc_pub = rospy.Publisher('controlCommand', controlCommand, queue_size=10)
	rospy.Subscriber('task_desiredAction', task_desiredAction, smach_data)
	rate = rospy.Rate(10)
	final_message = controlCommand()
	while not rospy.is_shutdown():
		final_message.yaw_set = 2
                final_message.pitch_set = 5
		final_message.depth_set = 10
		gnc_pub.publish(final_message)
		rate.sleep()

	else:
		exit()
