import rospy

from pose_planner import *
from marker_planner import *
from scan_planner import *

import numpy

from tf.transformations import euler_from_quaternion

def pose2d_from_param(robot_param):
  theta = euler_from_quaternion(robot_param['orientation'])[2]
  return robot_param['position'][:2] + [theta]

PI2 = numpy.pi/2

N_ROBOTS = 2

class MapPlanner():
  def __init__(self):
    rospy.init_node('map_planner')
    
    robot_params = rospy.get_param('robots')

    self.robots = [
      {
        'pose':   PosePlanner(None, 'robot_%02d/' % i),
        'marker': MarkerPlanner(None, 'robot_%02d/markers_0/' % i),
        'scan':   ScanPlanner(None, 'robot_%02d/scanner_0/' % i),
        'last_point': pose2d_from_param(robot_params[i])
      } for i in range(len(robot_params))
    ]

    rospy.Subscriber('/active', Bool, self.global_active_callback, queue_size=1)

  def global_active_callback(self, active):
    for robot in self.robots:
      robot['pose'].active_pub.publish(active)
      robot['marker'].active_pub.publish(active)
      robot['scan'].active_pub.publish(active)

  def move(self, r, point):
    rp = self.robots[r]['pose']
    rp.goal_blocking(points_to_path([self.robots[r]['last_point'],point]))
    self.robots[r]['last_point'] = point

  def scan(self, r=1, region=[(-0.4,-0.4),(0.06,0.41)], step=0.05, flip=False):
    rs = self.robots[r]['scan']
    rs.goal_blocking(path_to_scan(region_path(region,step), flip=flip))

  def mark(self, r, on=True):
    if on:
      self.robots[r]['marker'].markers_on(None)
    else:
      self.robots[r]['marker'].markers_off(None)

  def map_trajectory(self):
    # Initial pose fix
    self.mark(0)
    rospy.sleep(2)
    self.mark(0,False)
    rospy.sleep(0.2)

    # Scan 0.0
    self.scan(0)
    rospy.sleep(0.2)

    # move, turn, turn
    self.mark(0)
    self.move(0,(2.0,0.0,PI2+0.05))
    self.mark(0, False)
    rospy.sleep(0.2)

    self.mark(1)
    self.move(1,(1.0,2.0,0.05))
    self.mark(1,False)
    rospy.sleep(0.2)

    # Scan 1.0
    self.scan(1)
    rospy.sleep(0.2)

    # move, turn, turn
    self.mark(1)
    self.move(1,(3.0,2.0,-PI2-0.05))
    self.mark(1,False)
    rospy.sleep(0.2)

    self.mark(0)
    self.move(0,(2.0,0.0,-0.05))
    self.mark(0,False)
    rospy.sleep(0.2)

    # Scan 0.1
    self.scan(0,flip=True)
    rospy.sleep(0.2)

if __name__ == '__main__':
  mp = MapPlanner()
