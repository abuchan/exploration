#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from exploration.srv import *

import threading

# This controller will try to get the robot to the designated pose within
# the bounds specified by covariance by the time in the header timestamp
# If the time passes, the controller will cease

class PoseController:
  def __init__(self, rate = 10.0, stale_thresh=2.0):
    rospy.init_node('pose_controller')

    self.rate = rate
    self.stale_thresh
    self.stale_duration = rospy.Duration(self.stale_thresh/self.rate):
    self.rate_timer = rospy.Rate(self.rate)
    self.goals = []

    self.state = 'idle'
    self.pose_history = []
    self.last_goal = None

    self.history_length = 10

    self.lock = threading.Condition()

    self.current_goal_pub = rospy.Publisher(
      'current_goal', PoseWithCovarianceStamped, queue_size=1)

    self.cmd_vel_pub = rospy.Publisher(
      'cmd_vel', Twist, queue_size=1)

    rospy.Subscriber(
      'odometry', Odometry, self.last_pose_callback, queue_size=1)

    rospy.Service('clear_goals', ClearGoals, self.clear_goals)
    rospy.Service('add_goal', AddGoal, self.add_goal)

  def last_pose_callback(self, msg):
    self.lock.acquire()
    self.last_pose = msg
    self.lock.release()

  def clear_goals(self, request):
    self.lock.acquire()
    self.goals = []
    self.lock.release()
    return ClearGoalResponse()

  def add_goal(self, request):
    new_deadline = request.goal.header.stamp
    if new_deadline < rospy.Time.now():
      return AddGoalResponse(False)
    
    self.lock.acquire()

    for i in range(len(self.goals)):
      if new_deadline <= self.goals[i].header.stamp:
        break
    
    # If the new goal deadline matches one in the list, replace the old goal
    # with the new one
    if new_deadline == self.goals[i].header.stamp:
      self.goals[i] = request.goal

    # Otherwise insert it in order
    else:
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
    
  def control(self, current_pose, goal_pose):
    cmd = Twist()
    return cmd
  
  def safe(self, current_pose):

  def progress(self,

  def update_state(self, pose, goal):

    # IDLE: there is no goal
    if self.state == 'idle':
      self.last_goal = None
      self.pose_history = []

      if goal is not None:
        
        if pose is not None:
          self.pose_history = [pose]
          self.state = 'active'
        
        else:
          self.state = 'wait'
    
    # WAIT: there is a goal but no pose information
    elif self.state == 'wait':

      if goal is None:
        self.state = 'idle' 
      
      elif pose is not None:
        self.state = 'active'
    
    # ACTIVE: there is a goal and pose information. Apply control while
    # monitoring safety and progress
    elif self.state == 'active':
      
      if not self.safe(pose):
        self.state = 'unsafe'

      elif goal is None:
        self.state = 'idle'

      elif pose is None:
        self.state = 'wait'

      elif self.last_goal is not goal:
        self.pose_history = [pose]
        
      else:
        self.pose_history = [pose] + self.pose_history[:self.history_length]
      
        if not self.progress(self.pose_history, goal):
          if self.close(pose, goal):
            self.state = 'done'

          else:
            self.state = 'fail'

      self.last_goal = goal

    # UNSAFE: we have entered an unsafe state and need a new goal to leave it
    elif self.state == 'unsafe':
      self.pose_history = []
      
      if self.last_goal is not goal:
        self.pose_history = [pose]
        self.state = 'active'

      self.last_goal = goal

    # DONE: we stopped making progress and are close to the goal
    elif self.state == 'done':

      if goal is None:
        self.state = 'idle'
      
      elif self.last_goal is not goal:
        self.pose_history = [pose]
        self.state = 'active'
      
      self.last_goal = goal

    # FAIL: we stopped making progress and are not close to the goal
    elif self.state == 'fail':

      if goal is None:
        self.state = 'idle'
      
      elif self.last_goal is not goal:
        self.pose_history = [pose]
        self.state = 'active'

      self.last_goal = goal

  def run(self):
    while not rospy.is_shutdown():
      now = rospy.Time.now()

      self.lock.acquire()
      
      # Remove expired goals
      i = 0
      while i < len(goals) and goals[i].header.stamp < now:
        i += 1
      goals = goals[i:]

      # If we have a valid current goal, extract it for control
      if len(self.goals):
        current_goal = self.goals[0]
      else:
        current_goal = None
      
      # Grab current pose from last pose, unless it is stale
      if self.last_pose is not None and (now - self.last_pose.header.stamp) < self.stale_duration:
        current_pose = self.last_pose
      else:
        current_pose = None

      self.lock.release()
      
      # Publish current goal if it exists
      if current_goal is not None:
        self.current_goal_pub.publish(current_goal)
      
      # Update and publish state
      self.update_state(current_pose, current_goal)
      self.state_pub.publish(self.state)
  
      # Publish control action
      self.cmd_vel_pub.publish(self.control(current_pose, current_goal))

      self.rate_timer.sleep()

if __name__ == '__main__':
  pc = PoseController()
  pc.run()
