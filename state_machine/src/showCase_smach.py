import rospy
import smach
import smach_ros
from time import sleep

class Initialize_Robot(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('Initialize_Robot')
                return 'Success'

class SearchForBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('SearchForBuoy')
                return 'Success'

class CenterWithBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('CenterWithBuoy')
                return 'Success'

class LostBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('LostBuoy')
                return 'Success'

class BumpIntoBuoy(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('BumpIntoBuoy')
                return 'Success'

class SearchForBase(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success','Failed'])
        def execute(self, userdata):
                rospy.loginfo('SearchForBase')
                return 'Success'

class Surface(smach.State):
        def __init__(self):
                smach.State.__init__(self, outcomes=['Success'])
        def execute(self, userdata):
                rospy.loginfo('Surface')
                return 'Success'

def the_loop():
	sleep(3)
	rospy.init_node('my_state_machine')
	my_machine = smach.StateMachine(outcomes=['StopRobot'])
	with my_machine:
		smach.StateMachine.add('Initialize_Robot', Initialize_Robot(), transitions={'Success':'SearchForBuoy', 'Failed':'Surface'})
		smach.StateMachine.add('SearchForBuoy', SearchForBuoy(), transitions={'Success':'CenterWithBuoy', 'Failed':'Surface'})
		smach.StateMachine.add('CenterWithBuoy', CenterWithBuoy(), transitions={'Success':'BumpIntoBuoy', 'Failed':'LostBuoy'})
		smach.StateMachine.add('LostBuoy', LostBuoy(), transitions={'Success':'BumpIntoBuoy', 'Failed':'Surface'})
		smach.StateMachine.add('BumpIntoBuoy', BumpIntoBuoy(), transitions={'Success':'SearchForBase', 'Failed':'Surface'})
		smach.StateMachine.add('SearchForBase', SearchForBase(), transitions={'Success':'Surface', 'Failed':'Surface'})
		smach.StateMachine.add('Surface', Surface(), transitions={'Success':'StopRobot'})

	sis = smach_ros.IntrospectionServer('server', my_machine, '/Arens_State_Machine')
	sis.start()
	outcome = my_machine.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	the_loop()
