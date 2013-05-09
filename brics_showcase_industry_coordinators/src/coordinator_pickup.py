#!/usr/bin/env python
import roslib; roslib.load_manifest('brics_showcase_industry_coordinators')
import rospy

# protected region customHeaders on begin #
import smach
import smach_ros
import time
from brics_showcase_industry_interfaces.msg import PickUpAction, MoveArmCartAction, MoveArmCartGoal
from brics_showcase_industry_interfaces.srv import GetObjectPose, MoveGripper, MoveGripperRequest, FindObject
from actionlib import *
from actionlib.msg import *
from smach_ros import ActionServerWrapper
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseStamped
import PyKDL

# protected region customHeaders end #

class coordinator_pickup_impl:
  
  def __init__(self):
    # protected region initCode on begin #
    self.config_over_object = PoseStamped()
    genpy.message.fill_message_args(self.config_over_object, rospy.get_param('~over_object'))

    self.config_object = PoseStamped()
    genpy.message.fill_message_args(self.config_object, rospy.get_param('~object'))

    self.config_home = PoseStamped()
    genpy.message.fill_message_args(self.config_home, rospy.get_param('~home'))


    self.ps = PoseStamped()
    self.pshome = PoseStamped()
    self.ps.header.stamp = rospy.Time.now()

    self.open = MoveGripperRequest()
    self.open.open = 1

    self.close = MoveGripperRequest()
    self.close.open = 0
    self.detection_counter = 0
    # protected region initCode end #
    pass

  def findobjectcb(self, a, b):
    print "find object. get result from camera to world model"
    return "succeeded"

  def getposecb(self, a, b):
    print "Setting Pose based on world model"

    if((rospy.Time.now().to_sec() - b.pose.header.stamp.to_sec()) > 4.0):
      print "time now = ", rospy.Time.now().to_sec(), "time from pose = ", b.pose.header.stamp.to_sec()
      print "Time stamp of detection to old: diff=", (rospy.Time.now().to_sec() - b.pose.header.stamp.to_sec())
      if(self.detection_counter <= 4):
        self.detection_counter += 1
        return "preempted"
      else:
        self.detection_counter = 0
        return "aborted"

    self.config_over_object.header.stamp = rospy.Time.now()
    self.config_over_object.pose.position.x = b.pose.pose.position.x
    self.config_over_object.pose.position.y = b.pose.pose.position.y
    #ao = PyKDL.Rotation.Quaternion(self.config_over_object.pose.orientation.x, self.config_over_object.pose.orientation.y, self.config_over_object.pose.orientation.z, self.config_over_object.pose.orientation.w)
    #ao = PyKDL.Rotation.Quaternion(self.config_over_object.pose.orientation.x, self.config_over_object.pose.orientation.y, self.config_over_object.pose.orientation.z, self.config_over_object.pose.orientation.w)
    #bo = PyKDL.Rotation.Quaternion(b.pose.pose.orientation.x, b.pose.pose.orientation.y, b.pose.pose.orientation.z, b.pose.pose.orientation.w)
    #c = ao #* bo#.Inverse()
    m_manip = pm.toMatrix( pm.fromMsg(self.config_over_object.pose) )
    m_obj = pm.toMatrix( pm.fromMsg(b.pose.pose) )
    import numpy
    from tf.transformations import *
    m_ori = pm.toMatrix( pm.fromTf( ((0,0,0), quaternion_from_euler(0, 0, math.radians(-20.0))) ))
    box_pose = pm.toMsg( pm.fromMatrix(numpy.dot(m_manip,m_obj)) )
    m_test = numpy.dot(m_manip,m_obj)
    newrot = pm.toMsg( pm.fromMatrix(m_test))
    print newrot
    print self.config_over_object.pose
    #print c.GetQuaternion()
    self.config_over_object.pose.orientation = box_pose.orientation
    #self.config_over_object.pose.orientation.y = c.GetQuaternion()[1]
    #self.config_over_object.pose.orientation.z = c.GetQuaternion()[2]
    #self.config_over_object.pose.orientation.w = c.GetQuaternion()[3]
    #self.config_over_object.pose.position.z = 0.30#0.430

    self.config_object.header.stamp = rospy.Time.now()
    self.config_object.pose.position.x = b.pose.pose.position.x
    self.config_object.pose.position.y = b.pose.pose.position.y
    #self.config_object.pose.position.z = 0.2 #0.095
    self.config_object.pose.orientation = box_pose.orientation

  def configure(self):
    # protected region configureCode on begin #
    # Create a SMACH state machine
    
    sm0 = smach.StateMachine(outcomes=['succeeded','aborted','preempted'], input_keys = ['action_feedback','pose'], output_keys = ['action_feedback'])
    sis = smach_ros.IntrospectionServer('coordinator_pickup', sm0, '/pickup_sm')
    sis.start()
    with sm0:
      smach.StateMachine.add('FIND_OBJECT', smach_ros.ServiceState('/find_object', FindObject, response_cb=self.findobjectcb), transitions={'succeeded':'GET_POSE_FROM_WORLDMODEL', 'aborted':'aborted', 'preempted': 'FIND_OBJECT'})
      smach.StateMachine.add('GET_POSE_FROM_WORLDMODEL', smach_ros.ServiceState('/getObjectPose', GetObjectPose, response_cb=self.getposecb, input_keys = ['pose']), transitions={'succeeded':'MOVE_OVER_BOX', 'aborted':'aborted', 'preempted': 'GET_POSE_FROM_WORLDMODEL'})
      smach.StateMachine.add('MOVE_OVER_BOX', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_over_object)), {'succeeded':'MOVE_DOWN'})
      smach.StateMachine.add('MOVE_DOWN', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_object)), {'succeeded':'CLOSE_GRIPPER'})
      smach.StateMachine.add('CLOSE_GRIPPER', smach_ros.ServiceState('/MoveGripper', MoveGripper, request=self.close), transitions={'succeeded':'MOVE_UP', 'aborted':'MOVE_UP',})
      smach.StateMachine.add('MOVE_UP', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.config_over_object)), {'succeeded':'succeeded'})
      #smach.StateMachine.add('MOVE_HOME', smach_ros.SimpleActionState('MoveArmCart', MoveArmCartAction, goal = MoveArmCartGoal(pose_goal=self.pshome)), {'succeeded':'succeeded'})
    # Execute SMACH plan
    ActionServerWrapper(
          'pick_up',
          PickUpAction,
          wrapped_container = sm0,
          succeeded_outcomes = ['succeeded'],
          aborted_outcomes = ['aborted'],
          preempted_outcomes = ['preempted'],
      ).run_server()

    

    # protected region configureCode end #
    pass
  
  def update(self):
    # protected region updateCode on begin #
    # protected region updateCode end #
    pass
    
  

class coordinator_pickup:
  def __init__(self):
    self.impl = coordinator_pickup_impl()

  
    
  def run(self):
    self.impl.update()

if __name__ == "__main__":
  try:
    rospy.init_node('coordinator_pickup')
    rospy.sleep(2)
    n = coordinator_pickup()
    n.impl.configure()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        n.run()
        r.sleep()
      
  except rospy.ROSInterruptException:
    print "Exit"



