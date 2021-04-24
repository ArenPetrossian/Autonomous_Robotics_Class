#!/usr/bin/env python
import rospy
import smach
import smach_ros
from time import sleep

from ros_nodes.msg import sensors2smach
from ros_nodes.msg import lidar2smach
from ros_nodes.msg import cameras2smach
from ros_nodes.msg import controls2smach
from ros_nodes.msg import smach2controls

from initializeState import Initialize_Robot
from laneDetectState import Lane_Detection
from checkLeftRight import Turn_While_Detecting

rospy.init_node('my_state_machine')
smach_2_controls_pub = rospy.Publisher('smach2controls', smach2controls, queue_size=10)

class Forward3Feet(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('Forward3Feet')
                return 'Success'

class TurnParalleltoLane(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('TurnParalleltoLane')
                return 'Success'

class ForwardWhileDetecting(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Lane','Object','Pothole','Ramp','Finish_Line'])
        def execute(self, userdata):
                rospy.loginfo('ForwardWhileDetecting')
                return 'Finish_Line'

class ObjectAvoidance(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('ObjectAvoidance')
                return 'Success'

class PotholeAvoidance(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('PotholeAvoidance')
                return 'Success'

class RampClimbing(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('RampClimbing')
                return 'Success'

class DirectionCheck(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('DirectionCheck')
                return 'Success'

def the_loop():
	sleep(3)
	print("starting")
	#rospy.init_node('my_state_machine')
	#smach_2_controls_pub = rospy.Pubisher('smach2controls', smach2controls, queue_size=10)
	my_machine = smach.StateMachine(outcomes=['StopRobot'])
	with my_machine:
		smach.StateMachine.add('Initialize_Robot', Initialize_Robot(), transitions={'Success':'Lane_Detection', 'Failed':'Initialize_Robot'})
		smach.StateMachine.add('Lane_Detection', Lane_Detection(), transitions={'Success':'TurnParalleltoLane', 'Failed':'Turn_While_Detecting'})
		smach.StateMachine.add('Turn_While_Detecting', Turn_While_Detecting(), transitions={'Success':'TurnParalleltoLane', 'Failed':'Forward3Feet'})
		smach.StateMachine.add('Forward3Feet', Forward3Feet(), transitions={'Success':'Lane_Detection'})
		smach.StateMachine.add('TurnParalleltoLane', TurnParalleltoLane(), transitions={'Success':'ForwardWhileDetecting'})
		smach.StateMachine.add('ForwardWhileDetecting', ForwardWhileDetecting(), transitions={'Lane':'TurnParalleltoLane', 'Object':'ObjectAvoidance_Sub', 'Pothole':'PotholeAvoidance_Sub',
												      'Ramp':'RampClimbing_Sub', 'Finish_Line':'StopRobot'})
		smach.StateMachine.add('DirectionCheck', DirectionCheck(), transitions={'Success':'ForwardWhileDetecting'})

		avoidObj_sub = smach.StateMachine(outcomes=['Done'])
		with avoidObj_sub:
	                smach.StateMachine.add('ObjectAvoidance', ObjectAvoidance(), transitions={'Success':'Done'})
                smach.StateMachine.add('ObjectAvoidance_Sub', avoidObj_sub, transitions={'Done':'DirectionCheck'})

                avoidPot_sub = smach.StateMachine(outcomes=['Done'])
                with avoidPot_sub:
                        smach.StateMachine.add('PotholeAvoidance', PotholeAvoidance(), transitions={'Success':'Done'})
                smach.StateMachine.add('PotholeAvoidance_Sub', avoidPot_sub, transitions={'Done':'DirectionCheck'})

                ramp_sub = smach.StateMachine(outcomes=['Done'])
                with ramp_sub:
                        smach.StateMachine.add('RampClimbing', RampClimbing(), transitions={'Success':'Done'})
                smach.StateMachine.add('RampClimbing_Sub', ramp_sub, transitions={'Done':'DirectionCheck'})

	#avoidances and ramp
	sis = smach_ros.IntrospectionServer('server', my_machine, '/Arens_State_Machine')
	sis.start()
	outcome = my_machine.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	the_loop()
