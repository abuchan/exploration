#!/usr/bin/python

import rospy
from std_msgs.msg import String

from pose_planner import *
from marker_planner import *
from scan_planner import *

class Coordinator()
  def __init__(self, node_name='coordinator'):
    rospy.init_node(node_name)
    
    rospy.Subscriber('status',String,self.pose_callback,queue_size=1)
    self.pose_pub = rospy.Publisher('goal',Path,queue_size=1)

    rospy.Subscriber('markers_0/status',String,self.scan_callback,queue_size=1)
    self.marker_pub = rospy.Publisher('markers_0/goal',MarkerPath,queue_size=1)

    rospy.Subscriber('scanner_0/status',String,self.marker_callback,queue_size=1)
    self.scan_pub = rospy.Publisher('scanner_0/goal',ScanPath,queue_size=1)

    self.rate = rospy.Rate(5.0)

  def pose_callback(self, msg):
    self.pose_status = msg.data

  def scan_callback(self, msg):
    self.scan_status = msg.data

  def marker_callback(self, msg):
    self.marker_status = msg.data
  
  def move_mark_scan(self, points):
    self.pose_pub.publish(points_to_path(points))
    self.rate.sleep()
    while self.pose_status not in ['idle','done']:
      self.rate.sleep()

    self.marker_pub.publish(states_to_path(5*[PIXELS_SPECTRUM] + [PIXELS_BLACK]))
    self.rate.sleep()
    while self.marker_status not in ['idle','done']:
      self.rate.sleep()

    self.scan_pub.publish(path_to_scan(region_path([(-0.1,-0.15),(0.11,0.16)], 0.05)))
    self.rate.sleep()
    while self.scan_status not in ['idle','done']:
      self.rate.sleep()
  
  def run(self):
    while not rospy.is_shutdown():
      for i in range(len(SQUARE_POINTS)-1):
        self.move_mark_scan(SQUARE_POINTS[i,(i+2)])

if __name__ == '__main__':
  co = Coordinator()
  co.run()
