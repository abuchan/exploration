#!/usr/bin/python

import numpy

import rospy

from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException, Exception as tfException
from tf.transformations import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Vector3, TwistWithCovariance, Twist

def msg_to_numpy(msg):
  if type(msg) in (tuple,list):
    vals = msg
  else:
    vals = [msg.x, msg.y, msg.z]
    if 'w' in dir(msg):
      vals.append(msg.w)
  return numpy.array(vals)

def quats_to_twist(Q0, Q1):
  q1 = Q1.copy()
  if Q0.dot(Q1) < 0.0:  
    q1 *= -1
  dQ = quaternion_multiply(q1, quaternion_inverse(Q0))
  return 2*numpy.arccos(dQ[3]) / (1-dQ[3]**2)**0.5 * dQ[:3]

class MocapPoseEstimator():
  def __init__(self, node_name='mocap_pose_estimator'):
    rospy.init_node(node_name)
    self.tl = TransformListener()
    self.robot_frame = rospy.get_namespace().strip('/') + '_mocap'
    self.state_estimate_pub = rospy.Publisher('state_estimate',Odometry,queue_size=1)
    self.last_state_estimate = None
    self.pose_covariance = numpy.diag([0.001,0.001,0.001,0.01,0.01,0.01]).reshape(-1)
    self.twist_covariance = numpy.diag([0.01,0.01,0.01,0.1,0.1,0.1]).reshape(-1)

  def run(self):
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
      try:
        stamp = self.tl.getLatestCommonTime('world', self.robot_frame)
        pos, ori = self.tl.lookupTransform('world', self.robot_frame, stamp)
      except (LookupException, ConnectivityException, ExtrapolationException, tfException):
        continue
     
      mocap_odom = Odometry(
        Header(0, stamp, 'world'),
        self.robot_frame,
        PoseWithCovariance(Pose(Point(*pos), Quaternion(*ori)), self.pose_covariance),
        TwistWithCovariance(Twist(), self.twist_covariance),
      )

      if self.last_state_estimate is not None:
        
        dt = (stamp - self.last_state_estimate.header.stamp).to_sec()
        
        if dt > 0.0:
          pose_0 = self.last_state_estimate.pose.pose
          P0, O0, P1, O1 = [msg_to_numpy(msg) for msg in [pose_0.position, pose_0.orientation, pos, ori]]
          v_lin = (P1-P0)/dt
          v_ang = quats_to_twist(O0, O1)/dt
          mocap_odom.twist.twist = Twist(Vector3(*v_lin), Vector3(*v_ang))
          
      self.state_estimate_pub.publish(mocap_odom)
      
      self.last_state_estimate = mocap_odom

      rate.sleep()

if __name__ == '__main__':
  mpe = MocapPoseEstimator()
  mpe.run()
