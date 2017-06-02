#!/usr/bin/python

from controller import *

from exploration.msg import MarkerPath, MarkerState

PROGRESS_EPSILON = 0.01

class MarkerController(Controller):
  def __init__(self, node_name='marker_controller'):
    super(MarkerController, self).__init__(MarkerPath, MarkerState, MarkerState, node_name, 1, 0.1)

    self.state_estimate = MarkerState()
    self.power_matrix = numpy.array([[1.0, 0.0],[0.0, 0.0]])

  def goal_len(self):
    if self.goal is not None:
      return len(self.goal.states)
    else:
      return 0

  def update_progress(self):
    next_progress = self.progress

    if self.progress < self.goal_len() and self.subgoal_complete():
      next_progress = numpy.floor(self.progress) + PROGRESS_EPSILON
    
    return next_progress

  def update_command(self):
    next_command = None

    if self.active and not self.subgoal_complete():
      next_command = self.goal.states[int(self.progress)]
      self.state_estimate = next_command
      self.progress = numpy.ceil(self.progress)
    
    return next_command

  def update_power_state(self):
    marker_power = sum([ord(p) for p in self.state_estimate.pixels])
    return numpy.array([marker_power,1.0])

if __name__ == '__main__':
  mc = MarkerController()
  mc.run()
