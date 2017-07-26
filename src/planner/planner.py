import rospy

from std_msgs.msg import Bool, String

def empty_stamped_path(PathType, stamp=None):
  path = PathType()

  if stamp is None:
    stamp = rospy.Time.now()
  elif type(stamp) is float:
    stamp = rospy.Time(stamp)
  path.header.stamp = stamp

  return path

class Planner(object):
  def __init__(self, PathType, node_name='planner', robot_name=''):
    if node_name is not None:
      rospy.init_node(node_name)
      rospy.Subscriber('/active', Bool, self.global_active_callback, queue_size=1)

    self.PathType = PathType
    self.goal_pub = rospy.Publisher(robot_name+'goal', self.PathType, queue_size=1)
    self.active_pub = rospy.Publisher(robot_name+'active', Bool, queue_size=1)
    rospy.Subscriber(robot_name+'status', String, self.status_callback, queue_size=1)

    self.status = None
    self.rate = rospy.Rate(100.0)

  def goal_blocking(self, goal):
    self.goal_pub.publish(goal)
    while not rospy.is_shutdown() and self.status != 'active':
      self.rate.sleep()
    while not rospy.is_shutdown() and self.status == 'active':
      self.rate.sleep()
    return self.status

  def status_callback(self, status):
    self.status = status.data

  def global_active_callback(self, active):
    self.active_pub.publish(active)

  def empty_stamped_path(self, stamp=None):
    path = self.PathType()

    if stamp is None:
      stamp = rospy.Time.now()
    elif type(stamp) is float:
      stamp = rospy.Time(stamp)
    path.header.stamp = stamp
    
    return path
