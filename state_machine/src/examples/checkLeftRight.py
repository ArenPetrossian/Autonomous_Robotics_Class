#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from time import sleep
from ros_nodes.msg import cameras2smach
from ros_nodes.msg import controls2smach
from ros_nodes.msg import smach2controls
from Subscriber import Subscribe_to

#rospy.init_node('sm')
#smach_2_controls_pub = rospy.Publisher('smach2controls', smach2controls, queue_size=10)

class Turn_While_Detecting(smach.State):
	def __init__(self):
		print("starting")
		smach.State.__init__(self, outcomes=['Success', 'Failed'])

		self.cameras_sub = Subscribe_to('cameras2smach')
                self.controls_sub = Subscribe_to('controls2smach')
		self.flag = 0
		self.counter = 0
		self.controls_angle = smach2controls()
		sleep(2)

	def execute(self, userdata):
		self.cameras_data = self.cameras_sub.get_data()
		print (self.cameras_data)

		while (self.cameras_data.something == 0): #While there is no line detected
			sleep(0.01)
			self.cameras_data = self.cameras_sub.get_data()
			self.controls_data = self.controls_sub.get_data()
			print (self.cameras_data, self.controls_data)
			
			if (self.flag == 0):
				self.controls_angle.yaw_setpoint = -45
				smach_2_controls_pub.publish(self.controls_angle)
				self.flag = 1

			elif ((self.flag == 1) and (self.controls_data.stabilized == True) and (self.counter < 200)):
				self.counter = self.counter + 1
			elif ((self.flag == 1) and (self.counter == 200)):
				self.controls_angle.yaw_setpoint = 90
				smach_2_controls_pub.publish(self.controls_angle)
				self.flag = 2
				
			elif ((self.flag == 2) and (self.controls_data.stabilized == True) and (self.counter < 400)):
				self.counter = self.counter + 1
			elif ((self.flag == 2) and (self.counter == 400)):
				return 'Failed'

		return 'Success'

def code():
        main = smach.StateMachine(outcomes=['Done', 'Not_Done'])
        with main:
                smach.StateMachine.add('Turn_While_Detecting', Turn_While_Detecting(), transitions={ 'Success':'Done',
										'Failed':'Not_Done'})
        sis = smach_ros.IntrospectionServer('server', main, '/tester')
        sis.start()
        outcome = main.execute()
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
        code()


