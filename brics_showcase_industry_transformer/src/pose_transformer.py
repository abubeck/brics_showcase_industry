#!/usr/bin/env python
import roslib; roslib.load_manifest('brics_showcase_industry_transformer')
import rospy

# protected region customHeaders on begin #
from geometry_msgs.msg import PoseStamped, Pose
from brics_showcase_industry_interfaces.srv import SetObjectPose, SetObjectPoseRequest
import copy
import time
# protected region customHeaders end #

from geometry_msgs.msg import PoseArray 
from brics_showcase_industry_interfaces.srv import FindObject, FindObjectResponse, FindObjectRequest 


class pose_transformer_impl:
	in_CameraDetections = PoseArray()
	
	def	__init__(self):
		self.config_CameraPose = rospy.get_param('~CameraPose', "[[0,0,0],[0,0,0,0]]")
		self.config_MeterPerPixel = rospy.get_param('~MeterPerPixel', 0.000671)
		self.config_ResolutionX = rospy.get_param('~ResolutionX', 1600.0)
		self.config_ResolutionY = rospy.get_param('~ResolutionY', 1200.0)
		self.config_camera_base_link_offset_X = rospy.get_param('~camera_base_link_offset_X')
		self.config_camera_base_link_offset_Y = rospy.get_param('~camera_base_link_offset_Y')
		self.config_camera_base_link_flag_Y = rospy.get_param('~camera_base_link_flag_Y')
		# protected region initCode on begin #
		self.worldmodel_client = rospy.ServiceProxy('setObjectPose', SetObjectPose)
		self.out_CameraDetections = PoseArray()
		self.received_something = False
		# protected region initCode end #
		pass
	
	def	configure(self):
		# protected region configureCode on begin #
		# protected region configureCode end #
		pass
	
	def	update(self):
		# protected region updateCode on begin #
		in_CameraDetections = copy.deepcopy(self.in_CameraDetections)
		#print "Configs:", self.config_ResolutionX, ", ", self.config_ResolutionY
		#print "Detection:",in_CameraDetections 
		
		# check if detection is available
		if len(in_CameraDetections.poses) <= 0:
			rospy.logdebug("detection array empty")
			return
		
		# do transformation from pixel coords to camera_base_link coords in meter
		out1_CameraDetections = PoseArray()
		out1_CameraDetections.header = in_CameraDetections.header
		for pose in in_CameraDetections.poses:
			new_pose = Pose()
			new_pose.position.x = (pose.position.x - self.config_ResolutionX/2.0) * self.config_MeterPerPixel
			new_pose.position.y = (pose.position.y - self.config_ResolutionY/2.0) * self.config_MeterPerPixel
			new_pose.position.z = 0.0
			new_pose.orientation = pose.orientation
			out1_CameraDetections.poses.append(new_pose)
		
		# do transformation from camera_base_link to robot base_link
		out_CameraDetections = PoseArray()
		out_CameraDetections.header = out1_CameraDetections.header
		for pose in out1_CameraDetections.poses:
			new_pose = Pose()
			new_pose.position.x = self.config_camera_base_link_offset_X + pose.position.y
			new_pose.position.y = self.config_camera_base_link_offset_Y - pose.position.x * self.config_camera_base_link_flag_Y
			new_pose.position.z = 0.0
			new_pose.orientation = pose.orientation # TODO: rotate 180deg around x-axis
			out_CameraDetections.poses.append(new_pose)

		self.out_CameraDetections = out_CameraDetections
		# protected region updateCode end #
		pass
		
	def	callback_find_object(self, req):
		# protected region user implementation of service callback for find_object on begin #
		res = FindObjectResponse()
		
		# check if detection is available
		while(len(self.in_CameraDetections.poses) <= 0):
			rospy.loginfo("No Detection received yet")
			time.sleep(0.1)

		# writing pose to world model
		try:
			rospy.wait_for_service('setObjectPose', 1)
		except rospy.ROSException, e:
			print "%s"%e
			res.success = False
			return res
		try:
			req = SetObjectPoseRequest()
			req.pose.header.stamp = self.out_CameraDetections.header.stamp
			req.pose.header.frame_id = "/base_link"
			if len(self.out_CameraDetections.poses) <= 0:
				print "no detection result so far"
				res.success = False
				return res				
			req.pose.pose = self.out_CameraDetections.poses[0] ## HACK: only using first pose
			res_world_model = self.worldmodel_client(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			res.success = False
			return res
		rospy.loginfo("new pose sent to world model")


		res.success = True
		return res
		# protected region user implementation of service callback for find_object end #
		pass

	

class pose_transformer:
	def __init__(self):
		self.impl = pose_transformer_impl()
		self.CameraDetections = rospy.Subscriber("detected_pattern",PoseArray, self.CameraDetectionsCallback) 
		find_object_ = rospy.Service('find_object', FindObject, self.impl.callback_find_object)

	def CameraDetectionsCallback(self, a):
		self.impl.in_CameraDetections = a
	
		
	def run(self):
		self.impl.update()

if __name__ == "__main__":
	try:
		rospy.init_node('pose_transformer')
		n = pose_transformer()
		n.impl.configure()
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			n.run()
			r.sleep()
			
	except rospy.ROSInterruptException:
		print "Exit"



