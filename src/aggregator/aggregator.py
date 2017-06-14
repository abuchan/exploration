import rospy

from threading import Condition

def get_params_from_topic(topic):
  robots = rospy.get_param('robots')
  robot, child = topic.strip('/').split('/')[0:2]
  _, robot_index = robot.split('_')
  child_topic, child_index = child.split('_')
  children = robots[int(robot_index)]['children']
  child_params = filter(lambda c: c['topic'] == child_topic, children)[int(child_index)]
  return child_params

class Aggregator(object):
  def __init__(self, node_name, AggregateType, aggregate_topic, IndividualType, individual_topics, rate=10.0):
    rospy.init_node(node_name)
    self.AggregateType = AggregateType
    self.IndividualType = IndividualType
    
    self.lock = Condition()

    self.aggregate_pub = rospy.Publisher(aggregate_topic, self.AggregateType, queue_size=1)
    
    self.individual_states = {}

    for topic in individual_topics:
      self.individual_states[topic] = None

      rospy.Subscriber(topic, self.IndividualType, self.curried_callback(topic), queue_size=1)

    self.rate = rospy.Rate(rate)

  def is_active(self, state):
    return True

  def indiv_to_agg_state(self, state, topic):
    return state

  def curried_callback(self, ind_top):
    return lambda m: self.individual_callback(m, ind_top)

  def individual_callback(self, message, topic):
    self.lock.acquire()
    self.individual_states[topic] = self.indiv_to_agg_state(message, topic)
    self.lock.release()

  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      active_states = filter(self.is_active, self.individual_states.values())
      self.lock.release()

      aggregate = self.AggregateType(active_states)
      self.aggregate_pub.publish(aggregate)
      self.rate.sleep()
