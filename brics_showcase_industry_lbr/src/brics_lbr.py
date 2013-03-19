#!/usr/bin/env python
import roslib; roslib.load_manifest('brics_showcase_industry_lbr')
import rospy

# protected region customHeaders on begin #
import actionlib
from kinematics_msgs.srv import *
from brics_showcase_industry_interfaces.msg import MoveArmCartFeedback, MoveArmCartResult, MoveArmCartGoal, MoveArmCartAction
from trajectory_msgs.msg import *
from control_msgs.msg import *
from arm_navigation_msgs.srv import *
# protected region customHeaders end #



class brics_lbr_impl:
	
	def	__init__(self):
		# protected region initCode on begin
		rospy.wait_for_service("/environment_server/set_planning_scene_diff")
                self.envserv = rospy.ServiceProxy("/environment_server/set_planning_scene_diff", SetPlanningSceneDiff)
                rospy.sleep(1.0)
                self.envserv()

		self.iks = rospy.ServiceProxy('/silia_manipulator_kinematics/get_ik', GetPositionIK)
		self.client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
		# protected region initCode end #
		pass
	
	def	configure(self):
		# protected region configureCode on begin #
		# protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
		# protected region updateCode end #
		pass

	def callIKSolver(self, current_pose, goal_pose):
		rospy.sleep(2)
		rospy.wait_for_service("/brics_lbr_manipulator_kinematics/get_ik")
		iks = rospy.ServiceProxy('/brics_lbr_manipulator_kinematics/get_ik', GetPositionIK)
		rospy.sleep(2.0)
		rospy.wait_for_service("/brics_lbr_manipulator_kinematics/get_ik_solver_info")
		iks_info = rospy.ServiceProxy('/brics_lbr_manipulator_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
		rospy.sleep(2.0)
		info_res = iks_info()
		print info_res
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "arm_7_link"
		req.ik_request.ik_seed_state.joint_state.name = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		req.timeout = rospy.Duration(10.0)
		try:
			resp = iks(req)
		except:
			print "Didn't work"
			resp = iks(req)		
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

	def execute_MoveArmCart_callback(self, goal):
		_MoveArmCart_feedback = MoveArmCartFeedback()
		_MoveArmCart_result = MoveArmCartResult()
		# protected region updateCode on begin #

		print "Received MoveArmCartAction for ", goal.pose_goal
		(grasp_conf, error_code) = self.callIKSolver([1.3271463220572317, -1.18971404391667, 0.589597181844593, 0.6937175029311999, -0.5961144359180118, -1.3696334635724021, 1.69088276001348], goal.pose_goal)	
		#(grasp_conf, error_code) = self.callIKSolver([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], goal.pose_goal)	
		if(error_code.val != error_code.SUCCESS):
			rospy.logerr("Ik grasp Failed")
			print error_code
			self._as.set_aborted()
			return 'failed'
		# convert to ROS trajectory message
		print "Received IK result: ", grasp_conf
		joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
		traj_msg = JointTrajectory()
		traj_msg.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
		traj_msg.joint_names = joint_names
		point_msg = JointTrajectoryPoint()
		point_msg.positions = grasp_conf
		point_msg.velocities = [0]*len(joint_names)
		point_msg.time_from_start=rospy.Duration(3) # this value is set to 3 sec per point. \todo TODO: read from parameter
		traj_msg.points.append(point_msg)
		# sending goal
		client_goal = FollowJointTrajectoryGoal()
		client_goal.trajectory = traj_msg
		#print client_goal
		print "Sending Trajectory"
		self.client.send_goal(client_goal)
		self.client.wait_for_result()
		print "Finished movement"
		self._as.set_succeeded(_MoveArmCart_result)
		print "Finished overall"
		
		# protected region updateCode end #
		pass
		
	

class brics_lbr:
	def __init__(self):
		self.impl = brics_lbr_impl()
		self.impl._as = actionlib.SimpleActionServer("MoveArmCart", MoveArmCartAction, execute_cb=self.impl.execute_MoveArmCart_callback, auto_start=False)
		self.impl._as.start()

	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('brics_lbr')
		rospy.sleep(1.0)
		n = brics_lbr()
		n.impl.configure()
		while not rospy.is_shutdown():
			n.run()
			
	except rospy.ROSInterruptException:
		print "Exit"



