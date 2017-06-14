#!/usr/bin/python

import rospy
from planner import *
from exploration.msg import ScanPath, ScanPoint
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

import numpy

def grid_path(bounds, step=0.1):
  xmin, ymin, xmax, ymax = bounds
  xs = numpy.arange(xmin, xmax, step)
  ys = numpy.arange(ymin, ymax, step)
  xg, yg = numpy.meshgrid(xs,ys)
  for i in range(0,xg.shape[0],2):
    xg[i] = xg[i][::-1]
  return zip(xg.reshape(-1),yg.reshape(-1))
  
def region_path(region, step=0.1):
  region = numpy.array(region)
  xmin,ymin = region.min(axis=0)
  xmax,ymax = region.max(axis=0)
  return grid_path([xmin,ymin,xmax,ymax],step)

def path_to_scan(self, points, stamp=None):
  scan_path = empty_stamped_path(ScanPath,stamp)

  scan_index = 0
  for p in points:
    scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))
    scan_path.points.append(ScanPoint(scan_index+1, Vector3(p[0],p[1],1.0)))
    scan_index += 2
  scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))

  return scan_path

class ScanPlanner(Planner):
  def __init__(self, node_name = 'scan_planner'):
    super(ScanPlanner, self).__init__(ScanPath, node_name)

  def path_to_scan(self, points, stamp=None):
    scan_path = self.empty_stamped_path(stamp)
    
    scan_index = 0
    for p in points:
      scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))
      scan_path.points.append(ScanPoint(scan_index+1, Vector3(p[0],p[1],1.0)))
      scan_index += 2
    scan_path.points.append(ScanPoint(scan_index, Vector3(p[0],p[1],0.0)))

    return scan_path

  def scan_points(self, points):
    self.goal_pub.publish(self.path_to_scan(points))

  def scan_region(self, region=[(-0.1,-0.15),(0.11,0.16)], step=0.05):
    self.scan_points(region_path(region,step))

  def region_demo(self):
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
      self.scan_region()
      rate.sleep()

if __name__ == '__main__':
  sp = ScanPlanner()
  sp.region_demo()

