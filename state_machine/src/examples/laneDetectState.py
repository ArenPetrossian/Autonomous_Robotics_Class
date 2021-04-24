#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from time import sleep
from ros_nodes.msg import cameras2smach
from Subscriber import Subscribe_to

class Lane_Detection(smach.State):
	def __init__(self):
		print("starting")
		smach.State.__init__(self, outcomes=['Success', 'Failed'])
		self.cameras_sub = Subscribe_to('cameras2smach')
		self.counter = 0
		sleep(2)

	def execute(self, userdata):
		self.cameras_data = self.cameras_sub.get_data()
		print (self.cameras_data)

		while (self.cameras_data.something == 0):
			sleep(0.01)

			self.cameras_data = self.cameras_sub.get_data()
			print (self.cameras_data)
			if (self.counter > 500):
				return 'Failed'
			self.counter = self.counter + 1

		return 'Success'

def code():
        rospy.init_node('sm')
        main = smach.StateMachine(outcomes=['Done', 'Not_Done'])
        with main:
                smach.StateMachine.add('Lane_Detection', Lane_Detection(), transitions={ 'Success':'Done',
										'Failed':'Not_Done'})

        sis = smach_ros.IntrospectionServer('server', main, '/tester')
        sis.start()
        outcome = main.execute()
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
        code()


