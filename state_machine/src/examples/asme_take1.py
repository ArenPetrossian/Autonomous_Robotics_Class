import rospy
import smach
import smach_ros
from time import sleep

class Initialize_Robot(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Success','Fail'])
	def execute(self, userdata):
		rospy.loginfo('Initialize_Robot')
		return 'Success'

class Lane_Detection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Success','Fail'])
	def execute(self, userdata):
		rospy.loginfo('Lane_Detection')
		return 'Success'

class TurnWhileDetecting(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Fail'])
        def execute(self, userdata):
                rospy.loginfo('TurnWhileDetecting')
                return 'Success'

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
	sleep(4)
	rospy.init_node('my_state_machine')
	my_machine = smach.StateMachine(outcomes=['StopRobot'])
	with my_machine:
		smach.StateMachine.add('Initialize_Robot', Initialize_Robot(), transitions={'Success':'Lane_Detection', 'Fail':'Initialize_Robot'})
		smach.StateMachine.add('Lane_Detection', Lane_Detection(), transitions={'Success':'TurnParalleltoLane', 'Fail':'TurnWhileDetecting'})
		smach.StateMachine.add('TurnWhileDetecting', TurnWhileDetecting(), transitions={'Success':'TurnParalleltoLane', 'Fail':'Forward3Feet'})
		smach.StateMachine.add('Forward3Feet', Forward3Feet(), transitions={'Success':'Lane_Detection'})
		smach.StateMachine.add('TurnParalleltoLane', TurnParalleltoLane(), transitions={'Success':'ForwardWhileDetecting'})
		smach.StateMachine.add('ForwardWhileDetecting', ForwardWhileDetecting(), transitions={'Lane':'TurnParalleltoLane', 'Object':'ObjectAvoidance', 'Pothole':'PotholeAvoidance',
												      'Ramp':'RampClimbing', 'Finish_Line':'StopRobot'})
		smach.StateMachine.add('ObjectAvoidance', ObjectAvoidance(), transitions={'Success':'DirectionCheck'})
		smach.StateMachine.add('PotholeAvoidance', PotholeAvoidance(), transitions={'Success':'DirectionCheck'})
		smach.StateMachine.add('RampClimbing', RampClimbing(), transitions={'Success':'DirectionCheck'})
		smach.StateMachine.add('DirectionCheck', DirectionCheck(), transitions={'Success':'ForwardWhileDetecting'})

	sis = smach_ros.IntrospectionServer('server', my_machine, '/Arens_State_Machine')
	sis.start()
	outcome = my_machine.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	the_loop()
