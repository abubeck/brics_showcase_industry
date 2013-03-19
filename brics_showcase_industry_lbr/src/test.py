#!/usr/bin/env python
import roslib; roslib.load_manifest('brics_showcase_industry_lbr')
import rospy
# protected region customHeaders on begin #
import actionlib
from kinematics_msgs.srv import *
from geometry_msgs.msg import *
from brics_showcase_industry_interfaces.msg import MoveArmCartFeedback, MoveArmCartResult, MoveArmCartGoal, MoveArmCartAction

rospy.init_node('brics_lbr_test')
rospy.sleep(1.0)

ps = PoseStamped()
ps.header.stamp = rospy.Time.now()
ps.header.frame_id = "base_link"
psp = PoseStamped()
psp.header.stamp = rospy.Time.now()
psp.header.frame_id = "base_link"
psd = PoseStamped()
psd.header.stamp = rospy.Time.now()
psd.header.frame_id = "base_link"

#ps.pose.position.x = 0.9
#ps.pose.position.y = -0.5
#ps.pose.position.z = 1.3

#ps.pose.orientation.x = -0.007495
#ps.pose.orientation.y = -0.011300
#ps.pose.orientation.z = 0.887137
#ps.pose.orientation.w = 0.461307

#PICKUP LBR
psp.pose.position.x = 0.18
psp.pose.position.y = -0.716
psp.pose.position.z = 0.20

psp.pose.orientation.x = 0.545
psp.pose.orientation.y = 0.088
psp.pose.orientation.z = -0.011
psp.pose.orientation.w = -0.001

#DROP LBR
psd.pose.position.x = -0.22
psd.pose.position.y = -0.4
psd.pose.position.z = 0.17

psd.pose.orientation.x = 0.545
psd.pose.orientation.y = 0.088
psd.pose.orientation.z = -0.011
psd.pose.orientation.w = -0.001

print "Creating client"
client = actionlib.SimpleActionClient("/MoveArmCart", MoveArmCartAction)
rospy.sleep(1.0)
client_goal = MoveArmCartGoal()
client_goal.pose_goal = psp
print "Sensing goal"
client.send_goal(client_goal)
print "Waiting for result"
client.wait_for_result()
