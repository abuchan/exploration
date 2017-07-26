import rospy

from pose_planner import *
from marker_planner import *
from scan_planner import *

N_ROBOTS = 2

class MapPlanner():
  def __init__(self):
    rospy.init_node('map_planner')

    self.robots = [
      {
        'pose':   PosePlanner(None, 'robot_%02d/' % i),
        'marker': MarkerPlanner(None, 'robot_%02d/markers_0/' % i),
        'scan':   ScanPlanner(None, 'robot_%02d/scanner_0/' % i)
      } for i in range(N_ROBOTS)
    ]

    rospy.Subscriber('/active', Bool, self.global_active_callback, queue_size=1)

  def global_active_callback(self, active):
    for robot in self.robots:
      robot['pose'].active_pub.publish(active)
      robot['marker'].active_pub.publish(active)
      robot['scan'].active_pub.publish(active)

if __name__ == '__main__':
  mp = MapPlanner()
