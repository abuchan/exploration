#!/usr/bin/python

from planner import *

from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from tf.transformations import *

SQUARE_POINTS = [
  (1.0,0.0,0.0),
  (1.0,1.0,numpy.pi/2),
  (0.0,1.0,numpy.pi),
  (0.0,0.0,3*numpy.pi/2),
]
  
class PosePlanner(Planner):
  def __init__(self, node_name = 'pose_planner'):
    super(PosePlanner, self).__init__(Path, node_name)

  def elaborate_path(self, points):
    return points

  def execute_path(self, points, stamp=None):
    pose_path = self.empty_stamped_path(stamp)

    for x,y,theta in self.elaborate_path(points):
      pose_path.poses.append(
        PoseStamped(
          Header(),
          Pose(
            Point(x,y,0.0),
            Quaternion(*quaternion_about_axis(theta,(0,0,1)))
      )))

    self.goal_pub.publish(pose_path)

  def path_demo(self, points = SQUARE_POINTS):
    rate = rospy.Rate(0.04)
    while not rospy.is_shutdown():
      self.execute_path(points)
      rate.sleep()

if __name__ == '__main__':
  pp = PosePlanner()
  pp.path_demo()
