import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_srvs.srv import Trigger, TriggerRequest
from exploration.srv import AddGoal

from tf.transformations import *

class Planner:
  def __init__(self, node_name='planner', n_robots=1):
    rospy.init_node(node_name)
    self.rate = rospy.Rate(1)

    self.n_robots = n_robots
    self.odoms = [None] * n_robots
    self.states = [None] * n_robots
    self.clear_goals_svcs = []
    self.add_goal_svcs = []

    for r_i in range(n_robots):
      robot_ns = '/robot_%02d/' % r_i
      rospy.Subscriber(robot_ns + 'odometry', Odometry, self.curry(self.odom_callback, r_i), queue_size=1)
      rospy.Subscriber(robot_ns + 'state', String, self.curry(self.ctrl_state_callback, r_i), queue_size=1)
      self.clear_goals_svcs.append(rospy.ServiceProxy(robot_ns + 'clear_goals', Trigger))
      self.add_goal_svcs.append(rospy.ServiceProxy(robot_ns + 'add_goal', AddGoal))

  def curry(self, func, idx):
    return lambda x : func(x, idx)

  def odom_callback(self, msg, robot_idx):
    self.odoms[robot_idx] = msg

  def ctrl_state_callback(self, msg, robot_idx):
    self.states[robot_idx] = msg

  def robot_clear_goals(self, robot_idx):
    return self.clear_goals_svcs[robot_idx](TriggerRequest())
    
  def robot_add_goals(self, robot_idx, goals):
    rvals = []
    if type(goals) is not list:
      goals = [goals]
    for goal in goals:
      rvals.append(self.add_goal_svcs[robot_idx](goal))
    return rvals

  def time_in_future(self, time, now=None):
    if now is None:
      now = rospy.Time.now()
    return now + rospy.Duration(time)

  def xytt_to_odom(self, x, y, theta, time):
    odom = Odometry()
    odom.header.stamp = time
    odom.pose.pose.position = Point(x,y,0.0)
    odom.pose.pose.orientation = Quaternion(*quaternion_about_axis(theta, (0,0,1)))
    return odom

  def run(self):
    while not rospy.is_shutdown():
      rate.sleep()
