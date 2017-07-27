#!/usr/bin/python

from controller import *
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_matrix

def odom_mu_cov(odom, ref_O=None):
  P = odom.pose.position
  O = odom.pose.orientation

  # flip the quaternion so it is in the direction of the reference orientation
  O_sign = 1.0
  if ref_O is not None:
    O_sign = O.x * ref_O.x + O.y * ref_O.y + O.z * ref_O.z + O.w * ref_O.w
    
  mu = numpy.array([P.x, P.y, P.z, O_sign*O.x, O_sign*O.y, O_sign*O.z])

  if 'covariance' in dir(odom):
    cov = numpy.array(odom.covariance).reshape(6,-1)
  else:
    cov = None

  return mu, cov

def zscore_error(current_pose, goal_pose):
  mu_c, cov_c = odom_mu_cov(current_pose)
  mu_g, _ = odom_mu_cov(goal_pose)
  return numpy.linalg.inv(cov_g).dot(mu_c-mu_g)

def in_bounds(current_pose, goal_pose):
  _, bounds = odom_mu_cov(goal_pose)
  return abs(zscore_error(current_pose, goal_pose)) < bounds.diagonal()

# TODO: normalize by chi-squared function
def pose_likelihood(current_pose, goal_pose):
  mu_c, cov_c = odom_mu_cov(current_pose)
  mu_g, _ = odom_mu_cov(goal_pose, current_pose.pose.orientation)
  err = mu_c-mu_g
  return err.dot(numpy.linalg.inv(cov_c)).dot(err)

def tf_from_odom(odom):
  P = odom.pose.position
  O = odom.pose.orientation
  H = quaternion_matrix((O.x, O.y, O.z, O.w))
  H[:3,3] = P.x, P.y, P.z
  return H

def ab_control(pose, goal):
  cmd = Twist()

  H_p = tf_from_odom(pose)
  H_g = tf_from_odom(goal)
  H_d = numpy.linalg.inv(H_p).dot(H_g)
  T = H_d[:3,3]
  p = T.dot(T)**0.5

  # Apply mixed control only if distance is large enough to avoid instability
  if p > 0.001:
    R = H_d[:3,:3]
    d = T/p

    # alpha is the angle from robot x axis to vector towards goal
    a = numpy.arctan2(T[1],T[0])

    # beta is the angle from vector towards goal and goal x axis
    # (negative angle of vector towards goal in goal frame)
    d_g = R.T.dot(d)
    b = -numpy.arctan2(d_g[1], d_g[0])
    cmd.linear.x = 0.5 * p
    cmd.angular.z = 2 * a - 1.5 * b

  return cmd

def const_vw_control(pose, goal, v = 0.1, w = 0.5):
  cmd = Twist()
  
  H_p = tf_from_odom(pose)
  H_g = tf_from_odom(goal)
  H_d = numpy.linalg.inv(H_p).dot(H_g)
  T = H_d[:3,3]
  p = T.dot(T)**0.5
  
  if abs(H_d[1,0]) > 0.5:
    theta_sign = abs(H_d[1,0])/H_d[1,0]
  else:
    theta_sign = 2*H_d[1,0]
  
  if p > 0.01:

    cmd.linear.x = min(v, 1.0*p)

    a = numpy.arctan2(T[1],T[0])
    R = H_d[:3,:3]
    d_g = R.T.dot(T)
    b = -numpy.arctan2(d_g[1], d_g[0])
    
    cmd.angular.z = 2 * a - 0.1 * b
    
    if cmd.angular.z > w:
      cmd.angular.z = w
    if cmd.angular.z < -w:
      cmd.angular.z = -w
  
  else:
    cmd.angular.z = theta_sign * w

  return cmd

ROBOT_POWER_MATRIX = numpy.array([
  [14.0,   0.0, 1.68,  0.0],
  [ 0.0, 0.035,  0.0,  0.0],
  [1.68,   0.0,  0.0,  0.0],
  [ 0.0,   0.0,  0.0, 7.56]
])

class PoseController(Controller):
  def __init__(self, node_name='pose_controller'):
    super(PoseController, self).__init__(Path, Twist, Odometry, node_name, 3)

    self.power_matrix = ROBOT_POWER_MATRIX
    self.subgoal_distance = 0
    self.likelihood_pub = rospy.Publisher('likelihood', Float64, queue_size=1)
    self.state_estimate = None

  def goal_len(self):
    if self.goal is not None:
      return len(self.goal.poses)
    else:
      return 0

  def update_progress(self):
    next_progress = self.progress

    subgoal_index = int(self.progress)

    if self.state_estimate is not None and subgoal_index < self.goal_len():
      likelihood = pose_likelihood(self.state_estimate.pose, self.goal.poses[subgoal_index])
      while likelihood < 0.5 and subgoal_index < self.goal_len()-1:
        subgoal_index += 1
        self.progress = float(subgoal_index)
        likelihood = pose_likelihood(self.state_estimate.pose, self.goal.poses[subgoal_index])
      self.likelihood_pub.publish(likelihood)

      if self.subgoal_complete():
        self.subgoal_distance = likelihood
      
      subgoal_progress = (self.subgoal_distance - likelihood)/self.subgoal_distance
      
      if subgoal_progress < 0.001:
        subgoal_progress = 0.001

      # This shouldn't be possible, but in case of floating point error
      if likelihood < 0.1 or subgoal_progress > 1.0:
        subgoal_progress = 1.0

      next_progress = subgoal_index + subgoal_progress

    return next_progress

  def update_status(self):
    # TODO: switch to done if we have become unsafe or fail to make progress
    return super(PoseController, self).update_status()

  def update_command(self):
    next_command = Twist()
    
    subgoal_index = int(self.progress)

    if self.active and self.state_estimate is not None and subgoal_index < self.goal_len():
      #next_command = ab_control(self.state_estimate.pose, self.goal.poses[subgoal_index])
      next_command = const_vw_control(self.state_estimate.pose, self.goal.poses[subgoal_index])

    return next_command

  # Power state is [v_robot, w_robot, z_world * x_robot]
  def update_power_state(self):
    power_state = numpy.array([0.0, 0.0, 0.0, 1.0])

    if self.state_estimate is not None:
      H = tf_from_odom(self.state_estimate.pose)
      V = self.state_estimate.twist.twist.linear
      W = self.state_estimate.twist.twist.angular
      vw_world = numpy.array([[V.x, W.x],[V.y,W.y],[V.z,W.z],[0.0,0.0]])
      vw_robot = numpy.linalg.inv(H).dot(vw_world)
      power_state = numpy.array([vw_robot[0,0], vw_robot[2,1], H[2,0], 1.0])
    
    return power_state

if __name__ == '__main__':
  pc = PoseController()
  pc.run()
