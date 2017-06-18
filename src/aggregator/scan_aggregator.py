#!/usr/bin/python

from aggregator import *
from exploration.msg import ScanAggregate, ScanPoint
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

import sys
import numpy

from tf.transformations import *

class ScanAggregator(Aggregator):
  def __init__(self, node_name='scan_aggregator', scan_topics=None):
    if scan_topics is None:
      scan_topics = []

    super(ScanAggregator, self).__init__(node_name, ScanAggregate, '/active_scan', ScanPoint, scan_topics)

    self.configs = {st: None for st in scan_topics}

  def is_active(self, state):
    if state is not None:
      ori = state.pose.orientation
      quat = numpy.array([ori.x, ori.y, ori.z, ori.w])
      return quat.dot(quat) > 0.0
    else:
      return False

  def indiv_to_agg_state(self, state_msg, topic):
    if self.configs[topic] is None:
      child_params = get_params_from_topic(topic)
      self.configs[topic] = {
        'position': child_params['position'],
        'orientation': numpy.array(child_params['orientation']),
        'frame': topic.strip('/').split('/')[0]
      }

    frame = self.configs[topic]['frame']
    position = self.configs[topic]['position']

    # state_msg.point.z is power of laser, multiply by state estimate orientation
    # so magnitude of quaternion is power of laser
    Q0 = self.configs[topic]['orientation']
    laser_axis = numpy.array([state_msg.point.x, state_msg.point.y, 1.0])
    laser_axis_norm = laser_axis / (laser_axis.dot(laser_axis)**0.5)
    rot_axis = numpy.cross([0,0,1],laser_axis_norm)
    rot_angle = numpy.arcsin(rot_axis.dot(rot_axis)**0.5)
    Q1 = quaternion_about_axis(rot_angle, rot_axis)
    orientation = state_msg.point.z * quaternion_multiply(Q0,Q1)

    return PoseStamped(
      Header(0, rospy.Time.now(), frame),
      Pose(Point(*position), Quaternion(*orientation))
    )

if __name__ == '__main__':
  st_refs = filter(lambda s: len(s) == 3, sys.argv)
  st_refs = [tuple([int(r) for r in ref.split('.')]) for ref in st_refs]
  st = ['/robot_%02d/scanner_%01d/state_estimate' % ref for ref in st_refs]
  sa = ScanAggregator(scan_topics = st)
  sa.run()

