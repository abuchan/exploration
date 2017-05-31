#!/usr/bin/python

from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Kill, Spawn
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf.transformations import *

import rospy
import sys

class TurtlesimToOdomTF():
  def __init__(self, name='robot_00', x=5.0, y=5.0, theta=0.0):
    rospy.init_node('turtlesim_to_odom')
    self.name = name

    rospy.wait_for_service('/kill')
    kill = rospy.ServiceProxy('/kill',Kill)
    
    if name == 'robot_00':
      try:
        kill('turtle1')
      except:
        pass

    try:
      kill(name)
    except:
      pass

    x = float(x)
    y = float(y)
    theta = float(theta)

    rospy.wait_for_service('/spawn')
    spawn = rospy.ServiceProxy('/spawn',Spawn)
    try:
      spawn(x, y, theta, name)
    except:
      pass

    rospy.Subscriber('pose', TurtlePose, self.pose_callback, queue_size=1)
    self.odom_pub = rospy.Publisher('odometry', Odometry, queue_size=1)
    self.tfb = TransformBroadcaster()
  
  def pose_callback(self, msg):

    pos = (msg.x, msg.y, 0.0)
    ori = quaternion_about_axis(msg.theta, (0,0,1))
    now = rospy.Time.now()

    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = 'world'
    odom.child_frame_id = self.name
    odom.pose.pose = Pose(Point(*pos),Quaternion(*ori))
    self.odom_pub.publish(odom)

    self.tfb.sendTransform(pos, ori, now, self.name, 'world')

  def run(self):
    rospy.spin()
    
if __name__ == '__main__':
  # discard first arg (name of executable) and last two args (ros node name and log file)
  args = sys.argv[1:-2]
  ttotf = TurtlesimToOdomTF(*args)
  ttotf.run()
