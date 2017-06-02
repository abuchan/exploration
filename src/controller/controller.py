import rospy
from std_msgs.msg import String, Float64, Bool, Float64MultiArray

import numpy
from threading import Condition

# A controller is responsible for executing a sequence of commands to acheive
# a sequence of goal states with overall status control by a planner

# A Controller recieves a sequence of goal states (laser/galvo position, marker
# colors, pose) from the planner. It will execute towards the goals published 
# on the 'goal' topic in order by publishing to the 'command' topic when the
# 'active' topic is True.
class Controller(object):
  def __init__(self, GoalType, CommandType, StateEstimateType=None,
    node_name='controller', n_power_state=0, period=0.01):
    
    rospy.init_node(node_name)
    self.period = period
    self.rate = rospy.Rate(1.0/period)
    
    self.GoalType = GoalType
    self.CommandType = CommandType
    self.StateEstimateType = StateEstimateType

    self.lock = Condition()
    
    self.status = 'idle'
    self.goal = None
    self.new_goal = None
    self.progress = 0.0
    self.active = False

    self.status_pub = rospy.Publisher('status', String, queue_size=1)
    self.progress_pub = rospy.Publisher('progress', Float64, queue_size=1)
    rospy.Subscriber('active', Bool, self.active_callback, queue_size=1)
    
    self.command = None
    rospy.Subscriber('goal', self.GoalType, self.goal_callback, queue_size=1)
    self.command_pub = rospy.Publisher('command', self.CommandType, queue_size=1)
    if self.StateEstimateType is not None:
      self.state_estimate = None
      rospy.Subscriber('state_estimate', self.StateEstimateType,
        self.state_estimate_callback, queue_size=1)

    # Power state has power signal elements appended with 1 for affine value
    self.power_state = numpy.zeros(n_power_state+1)
    self.power_state[-1] = 1.0
    # TODO: Set power matrix based on rosparam
    self.power_matrix = numpy.zeros((n_power_state+1, n_power_state+1))
    self.energy_used = 0.0

    self.power_state_pub = rospy.Publisher('power_state', Float64MultiArray, queue_size=1)
    self.power_pub = rospy.Publisher('power', Float64, queue_size=1)
    self.energy_used_pub = rospy.Publisher('energy_used', Float64, queue_size=1)

  def goal_callback(self, goal):
    self.lock.acquire()
    self.new_goal = goal
    self.lock.release()
  
  # Return True if self.goal is allowed to be updated to self.new_goal
  # This determines whether in-progress moves can be aborted
  def goal_change_allowed(self):
    return True

  # Return True if a subgoal has been completed (progress has no fractional
  # component
  def subgoal_complete(self):
    return numpy.floor(self.progress) == self.progress

  def state_estimate_callback(self, state_estimate):
    self.lock.acquire()
    self.state_estimate = state_estimate
    self.lock.release()

  def active_callback(self, active):
    self.lock.acquire()
    self.active = active.data
    self.lock.release()

  # Return how many subgoals are in self.goal, should be implemented by subclass
  def goal_len(self):
    return 0

  # Progress update. This number must increase monotonically, and the integer part
  # represents the current goal index. This should be implemented by a subclass
  def update_progress(self):
    return 0.0

  # Main state machine update. Set next_status based on active and goals
  def update_status(self):
    next_status = self.status
    
    if self.goal_change_allowed() and self.new_goal is not None:
      self.goal = self.new_goal
      self.new_goal = None
      self.progress = 0.0
      self.active = False 
      next_status = 'idle'
    
    elif self.status == 'idle' and self.active and self.goal_len() > 0:
      next_status = 'active'

    elif self.status == 'active':
      if self.progress == self.goal_len():
        next_status = 'done'
      elif not self.active:
        next_status = 'idle'

    return next_status

  # Control to be implemented by subclass. Returns abstract CommandType message
  def update_command(self):
    return self.CommandType()

  # Return power state Numpy Array as a function of current state estimate and
  # and command. Should be implemented by subclass
  def update_power_state(self):
    return self.power_state

  # Apply quadratic multiplication to get power in watts, and accumulate to
  # energy_used. Should not need to change in subclass
  def update_power(self):
    self.power = self.power_state.dot(self.power_matrix).dot(self.power_state)
    self.energy_used += self.power * self.period

  def run(self):
    while not rospy.is_shutdown():
      # Do progress, status, command, and power_state update atomically
      self.lock.acquire()
      
      # Incorporate any state estimate into progress
      self.progress = self.update_progress()

      # Update main state machine based on progress, and active flag
      self.status = self.update_status()

      # Execute control based on state, progress, status
      self.command = self.update_command()

      # Compute power state based on progress, status, and command
      self.power_state = self.update_power_state()

      self.lock.release()

      # Set power_state, power, and energy_used
      self.update_power()

      # Publish status and power indicators
      self.status_pub.publish(self.status) 
      self.progress_pub.publish(self.progress)
      
      power_state_msg = Float64MultiArray()
      power_state_msg.data = tuple(self.power_state[:-1])
      self.power_state_pub.publish(power_state_msg)
      self.power_pub.publish(self.power)
      self.energy_used_pub.publish(self.energy_used)

      if self.command is not None:
        self.command_pub.publish(self.command)

      self.rate.sleep()

