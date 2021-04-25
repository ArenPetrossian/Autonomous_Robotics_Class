#! /usr/bin/env python
import rospy
from state_machine.msg import task_desiredAction
from sensing_and_actuation.msg import sensorInfo_actuatorStatus

def task_data(task_data):
        print(task_data)

def sensor_actuator_data(sensor_actuator_data):
	print (sensor_actuator_data)

while True:
	rospy.init_node('graphical_user_interface')
	rospy.Subscriber('task_desiredAction', task_desiredAction, task_data)
	rospy.Subscriber('sensorInfo_actuatorStatus', sensorInfo_actuatorStatus, sensor_actuator_data)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()

	else:
		exit()
