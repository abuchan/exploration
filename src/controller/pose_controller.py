#!/usr/bin/python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from exploration.srv import *
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import *

import threading

# This controller will try to get the robot to the designated pose within
# the bounds specified by covariance by the time in the header timestamp
# If the time passes, the controller will cease

class PoseController:
  def __init__(self, rate = 10.0, stale_thresh=2.0):
    rospy.init_node('pose_controller')

    self.rate = rate
    self.stale_duration = rospy.Duration(stale_thresh/self.rate)
    self.rate_timer = rospy.Rate(self.rate)
    self.goals = []
    self.sub_pose = None

    self.state = 'idle'
    self.pose_history = []
    self.last_goal = None
    self.recovering = False
    self.history_length = 10

    self.lock = threading.Condition()

    self.current_goal_pub = rospy.Publisher(
      'current_goal', Odometry, queue_size=1)

    self.state_pub = rospy.Publisher(
      'state', String, queue_size = 1)

    self.cmd_vel_pub = rospy.Publisher(
      'cmd_vel', Twist, queue_size=1)

    rospy.Subscriber(
      'odometry', Odometry, self.last_pose_callback, queue_size=1)

    rospy.Service('clear_goals', Trigger, self.clear_goals)
    rospy.Service('add_goal', AddGoal, self.add_goal)

  def last_pose_callback(self, msg):
    self.lock.acquire()
    self.sub_pose = msg
    self.lock.release()

  def clear_goals(self, request):
    self.lock.acquire()
    self.goals = []
    self.lock.release()
    return TriggerResponse(True, '')

  def add_goal(self, request):
    new_deadline = request.goal.header.stamp
    if new_deadline < rospy.Time.now():
      return AddGoalResponse(False)
    
    self.lock.acquire()

    i = 0
    while i < len(self.goals) and new_deadline > self.goals[i].header.stamp:
      i += 1
    
    # If the new goal deadline matches one in the list, replace the old goal
    # with the new one
    if i < len(self.goals) and new_deadline == self.goals[i].header.stamp:
      print 'Replacing goal %d' % i
      self.goals[i] = request.goal

    # Otherwise insert it in order
    else:
      print 'Inserting goal %d' % i
      self.goals.insert(i, request.goal)

    self.lock.release()

    return AddGoalResponse(True)
  
  def mu_cov(odom):
    P = odom.pose.position
    O = odom.pose.orientation
    mu = numpy.array([P.x, P.y, P.z, O.x, O.y, O.z])
    cov = numpy.array(odom.pose.covariance).reshape(6,-1)
    return mu, cov

  def error(self, current_pose, goal_pose):
    mu_c, cov_c = mu_cov(current_pose)
    mu_g, _ = mu_cov(goal_pose)
    return numpy.linalg.inv(cov_g).dot(mu_c-mu_g)

  def in_bounds(self, current_pose, goal_pose):
    _, bounds = mu_cov(goal_pose)
    return abs(self.error(current_pose, goal_pose)) < bounds.diagonal()

  def likelihood(self, current_pose, goal_pose):
    mu_c, cov_c = mu_cov(current_pose)
    mu_g, _ = mu_cov(goal_pose)
    err = mu_c-mu_g
    return err.dot(numpy.linalg.inv(cov_g)).dot(err)
    
  def tf_from_odom(self, odom):
    P = odom.pose.pose.position
    O = odom.pose.pose.orientation
    H = quaternion_matrix((O.x, O.y, O.z, O.w))
    H[:3,3] = P.x, P.y, P.z
    return H

  def control(self, pose, goal):
    cmd = Twist()
    if pose is not None and goal is not None and self.state == 'active':
      H_p = self.tf_from_odom(pose)
      H_g = self.tf_from_odom(goal)
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
        cmd.linear.x = 3 * p
        cmd.angular.z = 8 * a - 1.5 * b

    return cmd
  
  def safe(self, pose):
    return True

  def progress(self, pose_history, goal):
    return True

  def close(self, pose, goal):
    return False

  def update_state(self, pose, goal):

    # IDLE: there is no goal
    if self.state == 'idle':
      self.last_goal = None
      self.pose_history = []

      if goal is not None:
        if pose is not None:
          self.pose_history = [pose]
        self.state = 'active'
        
    # ACTIVE: there is a goal and pose information. Apply control while
    # monitoring safety and progress
    elif self.state == 'active':
      
      if pose is not None:
        self.pose_history = [pose] + self.pose_history[:self.history_length]
      
      if not self.safe(pose) and not (self.recovering and self.progress(self.pose_history, goal)):
        self.goals = []
        self.state = 'unsafe'

      elif goal is None:
        self.recovering = False
        self.state = 'idle'

      elif self.last_goal is not goal and pose is not None:
        self.pose_history = [pose]
        
      elif not self.progress(self.pose_history, goal):
        self.recovering = False

        if self.close(pose, goal):
          self.state = 'done'

        else:
          self.goals = []
          self.state = 'fail'

      self.last_goal = goal

    # UNSAFE: we have entered an unsafe state and need a new goal to leave it
    elif self.state == 'unsafe':
      
      if goal is not None:
        self.pose_history = []
        self.recovering = True
        if pose is not None:
          self.pose_history = [pose]
        self.state = 'active'
      
      self.last_goal = goal
    
    # DONE: we stopped making progress and are close to the goal
    elif self.state == 'done':

      if goal is None:
        self.state = 'idle'
      
      elif self.last_goal is not goal:
        if pose is not None:
          self.pose_history = [pose]
        self.state = 'active'
      
      self.last_goal = goal

    # FAIL: we stopped making progress and are not close to the goal. All
    # goals were removed by transition to this state, so we need a new goal
    # to move to ACTIVE
    elif self.state == 'fail':

      if goal is not None:
        self.state = 'active'
      
      self.last_goal = goal

  def run(self):
    while not rospy.is_shutdown():
      now = rospy.Time.now()

      self.lock.acquire()
      
      # Remove expired goals
      i = 0
      while i < len(self.goals) and self.goals[i].header.stamp < now:
        i += 1
      self.goals = self.goals[i:]

      # If we have a valid current goal, extract it for control
      if len(self.goals):
        current_goal = self.goals[0]
      else:
        current_goal = None
      
      # Grab current pose from last pose, unless it is stale
      if self.sub_pose is not None and (now - self.sub_pose.header.stamp) < self.stale_duration:
        current_pose = self.sub_pose
      else:
        current_pose = None
      
      # Update state machine
      self.update_state(current_pose, current_goal)
      
      self.lock.release()
      
      # Publish current goal if it exists
      if current_goal is not None:
        self.current_goal_pub.publish(current_goal)
      
      # Publish state
      self.state_pub.publish(self.state)
  
      # Publish control action
      self.cmd_vel_pub.publish(self.control(current_pose, current_goal))

      self.rate_timer.sleep()

if __name__ == '__main__':
  pc = PoseController()
  pc.run()
