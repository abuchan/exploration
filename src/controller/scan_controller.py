#!/usr/bin/python

from controller import *
from exploration.msg import ScanPath, ScanDelta, ScanPoint
from geometry_msgs.msg import Vector3

# Delay (in seconds) must be long enough to guarantee move is done, and image 
# is acquired from all scanners
DWELL_DELAY = 0.05
PROGRESS_EPSILON = 0.001

class ScanController(Controller):
  def __init__(self, node_name='scan_controller'):
    super(ScanController, self).__init__(ScanPath,ScanDelta,ScanPoint,node_name,2)
    
    # TODO: read galvo matrix from params
    self.galvo_matrix = numpy.eye(2)
    self.last_laser = 0.0
    self.state_estimate = ScanPoint(-1,Vector3())
    self.galvo_done_progress = -1.0
    self.power_matrix = numpy.eye(3)
    self.power_matrix[-1,-1] = 0

    self.state_estimate_pub = rospy.Publisher('state_estimate', ScanPoint, queue_size=1)

  def goal_len(self):
    if self.goal is not None:
      return len(self.goal.points)
    else:
      return 0

  def goal_change_allowed(self):
    return self.subgoal_complete()
  
  # Compute a ScanDelta to get from self.state_estimate to current subgoal
  def subgoal_state_to_delta(self):
    subgoal_idx = int(self.progress)
    subgoal_point = self.goal.points[subgoal_idx].point
    state_point = self.state_estimate.point
    delta = numpy.array([
      subgoal_point.x - state_point.x, subgoal_point.y - state_point.y
    ])
    galvo_delta = self.galvo_matrix.dot(delta)
    return ScanDelta(subgoal_idx, Vector3(galvo_delta[0], galvo_delta[1], subgoal_point.z))

  def update_progress(self):
    next_progress = self.progress
    subgoal_index = int(self.progress)
    now_sec = rospy.Time.now().to_sec()

    if self.subgoal_complete():

      # Update open-loop state estimate
      if self.progress != 0.0:
        self.state_estimate_pub.publish(self.goal.points[subgoal_index-1])
      
      # If there are more subgoals, and there is not a new goal update waiting,
      # start progress towards next subgoal
      if self.active and self.progress < self.goal_len() and self.new_goal is None:
        next_progress = subgoal_index + PROGRESS_EPSILON
        self.goal_start_time = now_sec
        delta = self.subgoal_state_to_delta()
        galvo_time = abs(delta.delta.x) + abs(delta.delta.y)
        self.goal_delta_time = galvo_time + DWELL_DELAY
        self.galvo_done_progress = subgoal_index + (galvo_time/self.goal_delta_time)
    
    else:
      next_progress = subgoal_index + min(1.0, (now_sec - self.goal_start_time)/self.goal_delta_time)
    
    return next_progress

  # Need to make sure galvo_done_progress is reset when new goal so power state
  # doesn't go high incorrectly
  def update_status(self):
    if self.goal_change_allowed() and self.new_goal is not None:
      self.galvo_done_progress = -1.0
    
    return super(ScanController,self).update_status()

  def update_command(self):
    next_command = None

    if not self.subgoal_complete():
      next_command = self.subgoal_state_to_delta()
      self.last_laser = next_command.delta.z
    
    return next_command
  
  def update_power_state(self):
    power_state = numpy.array([0.0,0.0,1.0])
    
    if self.progress < self.galvo_done_progress:
      power_state[0] = 1.0

    power_state[1] = self.last_laser
    
    return power_state

if __name__ == '__main__':
  sp = ScanController()
  sp.run()
