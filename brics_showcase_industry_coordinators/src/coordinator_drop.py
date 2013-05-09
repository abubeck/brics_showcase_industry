#!/usr/bin/env python
import roslib; roslib.load_manifest('brics_showcase_industry_coordinators')
import rospy

# protected region customHeaders on begin #
import smach
import smach_ros
import time
from brics_showcase_industry_interfaces.msg import DropDownAction, MoveArmCartAction, MoveArmCartGoal
from brics_showcase_industry_interfaces.srv import GetObjectPose, MoveGripper, MoveGripperRequest
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
from geometry_msgs.msg import PoseStamped
import genpy
# protected region customHeaders end #



class coordinator_drop_impl:
	
	def	__init__(self):
		# protected region initCode on begin #
		self.config_drop_over = PoseStamped()
		genpy.message.fill_message_args(self.config_drop_over, rospy.get_param('~drop_over'))

		self.config_drop = PoseStamped()
		genpy.message.fill_message_args(self.config_drop, rospy.get_param('~drop'))

		self.config_home = PoseStamped()
		genpy.message.fill_message_args(self.config_home, rospy.get_param('~home'))

		self.open = MoveGripperRequest()
		self.open.open = 1

		self.close = MoveGripperRequest()
		self.close.open = 0
		# protected region initCode end #
		pass
	
	def	configure(self):
		# protected region configureCode on begin #

		sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback'], output_keys = ['action_feedback'])
		sis = smach_ros.IntrospectionServer('coordinator_drop', sm0, '/drop_sm')
		sis.start()
		with sm0:
			smach.StateMachine.add('MOVE_OVER_BOX', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_drop_over)), {'succeeded':'MOVE_DOWN'})
			smach.StateMachine.add('MOVE_DOWN', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_drop)), {'succeeded':'OPEN_GRIPPER'})
			smach.StateMachine.add('OPEN_GRIPPER', smach_ros.ServiceState('/MoveGripper', MoveGripper, request=self.open), transitions={'succeeded':'MOVE_UP', 'aborted':'MOVE_UP',})
			smach.StateMachine.add('MOVE_UP', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_drop_over)), {'succeeded':'MOVE_HOME'})
			smach.StateMachine.add('MOVE_HOME', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_home)), {'succeeded':'succeeded'})

		# Execute SMACH plan
		ActionServerWrapper(
        	'drop_down',
        	DropDownAction,
        	wrapped_container = sm0,
        	succeeded_outcomes = ['succeeded'],
        	aborted_outcomes = ['aborted'],
        	preempted_outcomes = ['preempted'],
			).run_server()

		# protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
		# protected region updateCode end #
		pass
		
	

class coordinator_drop:
	def __init__(self):
		self.impl = coordinator_drop_impl()

	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('coordinator_drop')
		n = coordinator_drop()
		n.impl.configure()
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			n.run()
			r.sleep()
			
	except rospy.ROSInterruptException:
		print "Exit"



