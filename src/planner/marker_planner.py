#!/usr/bin/python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from planner import *
from exploration.msg import MarkerPath, MarkerState

#Colors in BGR order (i.e. color[0] = B)
COLOR_BLACK   = [   0,   0,   0]
COLOR_RED     = [   0,   0, 255]
COLOR_YELLOW  = [   0, 255, 255]
COLOR_GREEN   = [   0, 255,   0]
COLOR_CYAN    = [ 255, 255,   0]
COLOR_BLUE    = [ 255,   0,   0]
COLOR_MAGENTA = [ 255,   0, 255]
COLOR_WHITE   = [ 255, 255, 255]

PIXELS_SPECTRUM = [COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA]
PIXELS_BLACK    = 6*[COLOR_BLACK]
PIXELS_WHITE    = 6*[COLOR_WHITE]

def states_to_path(self, pixel_states=[PIXELS_SPECTRUM], stamp=None):
  marker_path = empty_stamped_path(MarkerPath,stamp)
  marker_path.states = [MarkerState(reduce(list.__add__,state)) for state in pixel_states]
  return marker_path

class MarkerPlanner(Planner):
  def __init__(self, node_name='marker_planner', robot_name=''):
    super(MarkerPlanner, self).__init__(MarkerPath, node_name, robot_name)
    rospy.Service(robot_name+'markers_on', Trigger, self.markers_on)
    rospy.Service(robot_name+'markers_off', Trigger, self.markers_off)

  def markers_on(self, req):
    self.set_markers([PIXELS_SPECTRUM])
    return TriggerResponse()

  def markers_off(self, req):
    self.set_markers([PIXELS_BLACK])
    return TriggerResponse()
  
  def states_to_path(self, pixel_states=[PIXELS_SPECTRUM], stamp=None):
    marker_path = self.empty_stamped_path(stamp)
    marker_path.states = [MarkerState(reduce(list.__add__,state)) for state in pixel_states]
    return marker_path

  def set_markers(self, pixel_states=[PIXELS_SPECTRUM], stamp=None):
    self.goal_pub.publish(self.states_to_path(pixel_states,stamp))
  
  def blink_demo(self):
    rate = rospy.Rate(0.5)
    blink = 10 * [PIXELS_SPECTRUM] + 8 * [PIXELS_BLACK]
    while not rospy.is_shutdown():
      self.set_markers(blink)
      rate.sleep()

  def spectrum_demo(self):
    rotation = []
    for i in range(6):
      rotation.extend(5*[PIXELS_SPECTRUM[i:] + PIXELS_SPECTRUM[:i]])
    rotation.extend([PIXELS_BLACK])
    rate = rospy.Rate(1.0/4.0)
    while not rospy.is_shutdown():
      self.set_markers(rotation)
      rate.sleep()
  
  def on_demo(self):
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
      self.set_markers([PIXELS_SPECTRUM])
      rate.sleep()

if __name__ == '__main__':
  mp = MarkerPlanner()
  rate = rospy.Rate(10.0)
  while not rospy.is_shutdown():
    rate.sleep()
  #mp.on_demo()
