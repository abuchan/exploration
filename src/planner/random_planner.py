#!/usr/bin/python

from planner import *
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

import numpy
import sys

from tf.transformations import *

class RandomPlanner(Planner): 
  def __init__(self, node_name='random_planner', n_robots=1):
    super(RandomPlanner, self).__init__(node_name, n_robots)
    self.running = True
    self.running_pub = rospy.Publisher(
      'running', Bool, queue_size = 1)
    
    rospy.Service('set_running', SetBool, self.set_running_callback)

    self.rng = numpy.random.RandomState(0x42)

  def set_running_callback(self, request):
    self.running = request.data
    return SetBoolResponse(True, '')

  def random_goal(self, deadline=20):
    pos = self.rng.uniform(0,10,3)
    ori = quaternion_about_axis(self.rng.uniform(-numpy.pi, numpy.pi),(0,0,1))
    goal = Odometry()
    goal.header.stamp = self.time_in_future(deadline)
    print deadline, (goal.header.stamp - rospy.Time.now()).to_sec(), (self.time_in_future(deadline) - rospy.Time.now()).to_sec()
    goal.pose.pose.position = Point(*pos)
    goal.pose.pose.orientation = Quaternion(*ori)
    return goal
    
  def run(self):
    while not rospy.is_shutdown():
      self.running_pub.publish(self.running)
      
      if self.running:
        for i in range(self.n_robots):
          if self.states[i] == 'idle':
            goals = 3*[None]
            for j in range(3):
              goals[j] = self.random_goal((j+1)*10.0)
            self.robot_add_goals(i, goals)

      self.rate.sleep()

if __name__ == '__main__':
  rp = RandomPlanner(n_robots=int(sys.argv[1]))
  rp.run()
