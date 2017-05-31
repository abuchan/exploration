#!/usr/bin/python

import rospy
from exploration.msg import ScanPath, ScanPoint
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

import numpy

class ScanPlanner():
  def __init__(self, node_name = 'scan_planner'):
    rospy.init_node(node_name)
    self.rate = rospy.Rate(1)

    self.goal_pub = rospy.Publisher('goal', ScanPath, queue_size=1)
    self.active_pub = rospy.Publisher('active', Bool, queue_size=1)
    rospy.Subscriber('/active', Bool, self.global_active_callback, queue_size=1)

  def global_active_callback(self, active):
    self.active_pub.publish(active)

  def path_to_scan(self, points, stamp=None):
    scan_path = ScanPath()

    if stamp is None:
      stamp = rospy.Time.now()
    elif type(stamp) is float:
      stamp = rospy.Time(stamp)
    scan_path.header.stamp = stamp
    
    scan_index = 0
    for p in points:
      scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))
      scan_path.points.append(ScanPoint(scan_index+1, Vector3(p[0],p[1],1.0)))
      scan_index += 2
    scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))

    return scan_path

  def scan_points(self, points):
    self.goal_pub.publish(self.path_to_scan(points))

  def region_path(self, region, step=0.1):
    region = numpy.array(region)
    xmin,ymin = region.min(axis=0)
    xmax,ymax = region.max(axis=0)
    return self.grid_path([xmin,ymin,xmax,ymax],step)

  def scan_region(self, region=[(-0.15,-0.1),(0.16,0.11)], step=0.1):
    self.scan_points(self.region_path(region,step))

  def grid_path(self, bounds, step=0.1):
    xmin, ymin, xmax, ymax = bounds
    xs = numpy.arange(xmin, xmax, step)
    ys = numpy.arange(ymin, ymax, step)
    xg, yg = numpy.meshgrid(xs,ys)
    for i in range(0,xg.shape[0],2):
      xg[i] = xg[i][::-1]
    return zip(xg.reshape(-1),yg.reshape(-1))
    
