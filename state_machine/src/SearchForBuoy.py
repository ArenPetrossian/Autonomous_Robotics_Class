#!/usr/bin/env python
import rospy
import smach
import smach_ros
import time
from computer_vision.msg import target
from guidance_navigation_control.msg import task_desiredAction
from guidance_navigation_control.msg import sensorInfo_actuatorStatus
from Subscriber import Subscribe_to


class SearchForBuoy(smach.State):
	def __init__(self):
		print("searching")
		smach.State.__init__(self, outcomes=['Success', 'Failed'])
		self.smach_pub = rospy.Publisher('task_desiredAction', task_desiredAction, queue_size=10)
		self.cv_sub = Subscribe_to('target')
		self.sensors_sub = Subscribe_to('sensorInfo_actuatorStatus')
		self.counter = 0
		self.task = task_desiredAction()
		time.sleep(2)

	def execute(self, userdata):
		#Assume Robot starts with all setpoints at 0
		print("Dive")
		self.task.depth_set = 1
		self.smach_pub.publish(self.task)
		time.sleep(0.5)
		self.sensors_data = self.sensors_sub.get_data()
		while not self.sensors_data.stabilized:
			time.sleep(0.01)
			self.counter = self.counter + 1
			if (self.counter > 1500):
				return 'Failed'
			self.sensors_data = self.sensors_sub.get_data()

		#Turn 45
		print("Turn 45")
		self.counter = 0
		self.task.depth_set = 0
		self.task.yaw_set = 45
		self.smach_pub.publish(self.task)
		time.sleep(0.5)
		self.sensors_data = self.sensors_sub.get_data()
		while not self.sensors_data.stabilized:
			time.sleep(0.01)
			self.counter = self.counter + 1
			if (self.counter > 1500):
				return 'Failed'
			self.sensors_data = self.sensors_sub.get_data()

		#Forward until see Buoy
		print("Forwards")
		self.counter = 0
		self.task.yaw_set = 0
		self.task.distance_set = 5
		self.smach_pub.publish(self.task)
		self.cv_data = self.cv_sub.get_data()
		while not self.cv_data.buoy1:
			time.sleep(0.01)
			self.counter = self.counter + 1
			if (self.counter > 2000):
				return 'Failed'
			self.cv_data = self.cv_sub.get_data()

		print("Buoy Found")
		self.task.distance_set = 0
		self.smach_pub.publish(self.task)
		return 'Success'


def code():
        rospy.init_node('sm')
        main = smach.StateMachine(outcomes=['Done', 'Not_Done'])
        with main:
                smach.StateMachine.add('SearchForBuoy', SearchForBuoy(), transitions={ 'Success':'Done',
										'Failed':'Not_Done'})

        sis = smach_ros.IntrospectionServer('server', main, '/tester')
        sis.start()
        outcome = main.execute()
        sis.stop()
        rospy.spin()


if __name__ == '__main__':
	code()
