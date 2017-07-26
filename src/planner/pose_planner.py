#!/usr/bin/python

from planner import *

from std_msgs.msg import Header, String
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from tf.transformations import *

import numpy

SQUARE_POINTS = [
  (0.0,0.0,0.0),
  (1.0,0.0,1.0),
  (1.0,1.0,2.0),
  (0.0,1.0,4.0),
  (0.0,0.0,0.0),
]

def elaborate_path(points):
  elaborated_points = []
  for P0, P1 in zip(points[:-1], points[1:]):
    path_theta = numpy.arctan2(P1[1]-P0[1],P1[0]-P0[0])
    PA = (P0[0], P0[1], path_theta)
    PB = (P1[0], P1[1], path_theta)
    elaborated_points.extend([P0,PA,PB,P1])
  return elaborated_points

def points_to_path(points, stamp=None):
  pose_path = empty_stamped_path(Path,stamp)

  for x,y,theta in elaborate_path(points):
    pose_path.poses.append(
      PoseStamped(
        Header(),
        Pose(
          Point(x,y,0.0),
          Quaternion(*quaternion_about_axis(theta,(0,0,1)))
    )))

  return pose_path

class PosePlanner(Planner):
  def __init__(self, node_name = 'pose_planner', robot_name=''):
    super(PosePlanner, self).__init__(Path, node_name, robot_name)
    self.status = 'idle'

  def execute_path(self, points, stamp=None):
    pose_path = self.empty_stamped_path(stamp)

    for x,y,theta in elaborate_path(points):
      pose_path.poses.append(
        PoseStamped(
          Header(),
          Pose(
            Point(x,y,0.0),
            Quaternion(*quaternion_about_axis(theta,(0,0,1)))
      )))

    self.goal_pub.publish(pose_path)

  def status_callback(self, msg):
    self.status = msg.data

  def path_demo(self, points = SQUARE_POINTS):
    rate = rospy.Rate(1.0)
    rospy.Subscriber('status',String,self.status_callback,queue_size=1)
    while not rospy.is_shutdown():
      if self.status in ['idle','done']:
        self.execute_path(points)
      rate.sleep()

if __name__ == '__main__':
  pp = PosePlanner()
  pp.path_demo()
