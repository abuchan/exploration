import rospy

from std_msgs.msg import Bool

def empty_stamped_path(PathType, stamp=None):
  path = PathType()

  if stamp is None:
    stamp = rospy.Time.now()
  elif type(stamp) is float:
    stamp = rospy.Time(stamp)
  path.header.stamp = stamp

  return path

class Planner(object):
  def __init__(self, PathType, node_name='planner'):
    rospy.init_node(node_name)

    self.PathType = PathType
    self.goal_pub = rospy.Publisher('goal', self.PathType, queue_size=1)
    self.active_pub = rospy.Publisher('active', Bool, queue_size=1)
    rospy.Subscriber('/active', Bool, self.global_active_callback, queue_size=1)

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
