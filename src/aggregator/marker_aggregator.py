#!/usr/bin/python

from aggregator import *
from exploration.msg import MarkerAggregate, MarkerStateStamped, MarkerPositionState, MarkerState

from std_msgs.msg import Header
from geometry_msgs.msg import Point

import sys

from tf.transformations import *

class MarkerAggregator(Aggregator):
  def __init__(self, node_name='marker_aggregator', marker_topics=None):
    if marker_topics is None:
      marker_topics = []
    
    super(MarkerAggregator, self).__init__(node_name, MarkerAggregate, '/active_markers', MarkerState, marker_topics, rate=1.0)
    
    self.configs = {mt: None for mt in marker_topics}

  def is_active(self, state):
    if state is not None:
      pixel_sums = [sum([ord(c) for c in pos_state.state]) for pos_state in state.marker_states]
      return sum(pixel_sums) > 0
    else:
      return False

  def indiv_to_agg_state(self, state_msg, topic):
    if self.configs[topic] is None:
      child_params = get_params_from_topic(topic)
      H = quaternion_matrix(child_params['orientation'])
      H[:3,3] = child_params['position']
      marker_positions = numpy.array(
        [[p['x'], p['y'], p['z'], 1.0] for p in child_params['marker_positions']]
      ).T
      transformed_positions = H.dot(marker_positions)[:3,:].T
      self.configs[topic] = {
        'positions':transformed_positions,
        'frame':topic.strip('/').split('/')[0]
      }
    
    frame = self.configs[topic]['frame']
    positions = self.configs[topic]['positions']
    states = [state_msg.pixels[(3*i):(3*i+3)] for i in range(len(state_msg.pixels)/3)]

    return MarkerStateStamped(
      Header(0, rospy.Time.now(), frame),
      [ MarkerPositionState(Point(*pos), state) for pos, state in zip(positions, states) ]
    )

if __name__ == '__main__':
  mt_refs = filter(lambda s: len(s) == 3, sys.argv)
  mt_refs = [tuple([int(r) for r in ref.split('.')]) for ref in mt_refs]
  mt = ['/robot_%02d/markers_%01d/state_estimate' % ref for ref in mt_refs]
  ma = MarkerAggregator(marker_topics = mt)
  ma.run()
