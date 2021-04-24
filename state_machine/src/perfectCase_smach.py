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

class SearchForBuoy(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Success'])
	def execute(self, userdata):
		rospy.loginfo('SearchForBuoy')
		return 'Success'

class CenterWithBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('CenterWithBuoy')
                return 'Success'

class BumpIntoBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('BumpIntoBuoy')
                return 'Success'

class SearchForBase(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('SearchForBase')
                return 'Success'

class Surface(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Finished'])
        def execute(self, userdata):
                rospy.loginfo('Surface')
                return 'Finished'

def the_loop():
	sleep(3)
	rospy.init_node('my_state_machine')
	my_machine = smach.StateMachine(outcomes=['StopRobot'])
	with my_machine:
		smach.StateMachine.add('Initialize_Robot', Initialize_Robot(), transitions={'Success':'SearchForBuoy', 'Fail':'StopRobot'})
		smach.StateMachine.add('SearchForBuoy', SearchForBuoy(), transitions={'Success':'CenterWithBuoy'})
		smach.StateMachine.add('CenterWithBuoy', CenterWithBuoy(), transitions={'Success':'BumpIntoBuoy'})
		smach.StateMachine.add('BumpIntoBuoy', BumpIntoBuoy(), transitions={'Success':'SearchForBase'})
		smach.StateMachine.add('SearchForBase', SearchForBase(), transitions={'Success':'Surface'})
		smach.StateMachine.add('Surface', Surface(), transitions={'Finished':'StopRobot'})

	sis = smach_ros.IntrospectionServer('server', my_machine, '/Arens_State_Machine')
	sis.start()
	outcome = my_machine.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	the_loop()
