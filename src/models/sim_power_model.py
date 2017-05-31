#!/usr/bin/python

import rospy

from ros_zumy.srv import SetLaserGalvo, SetLaserGalvoResponse
from std_msgs.msg import Float32, Float32MultiArray
from nav_msgs.msg import Odometry

import threading

from tf.transformations import *

def odom_to_tf_vel(odom):
  pos = odom.pose.pose.position
  ori = odom.pose.pose.orientation
  
  H = quaternion_matrix(ori.x, ori.y, ori.z, ori.w)
  H[:3,3] = (pos.x, pos.y, pos.z)
  
  vl = o.twist.twist.linear
  va = o.twist.twist.angular
  
  v_lin = numpy.array([vl.x, vl.y, vl.z, 1.0])
  v_ang = numpy.array([va.x, va.y, va.z, 1.0])

  return H, v_lin, v_ang

class SimPowerModel():
  def __init__(self):
    rospy.init_node('sim_power_model')
    rospy.Service('set_laser_galvo', SetLaserGalvo, self.laser_galvo_callback)
    self.power_state_pub = rospy.Publisher('power_state', Float32MultiArray, queue_size=1)
    self.power_pub = rospy.Publisher('power', Float32, queue_size=1)
    self.energy_used_pub = rospy.Publisher('energy_used', Float32, queue_size=1)
    
    
    # v_xr, omega_zr, v_zw, markers, laser, galvo, affine one
    self.power_state = numpy.zeros(7)
    self.power_state[-1] = 1
    self.power_coeff = numpy.eye(7)
    self.dt = 0.01
    self.rate = rospy.Rate(1/self.dt)
    self.galvo_time = 0.0
    self.energy_used = 0.0

    self.lock = threading.Condition()

  def odom_callback(self, odom):
   H, v_lin, v_ang = odom_to_tf_vel(odom)
   v_xr = v_lin[0]
   omega_zr = v_ang[3]
   v_w = numpy.linalg.inv(H).dot(v_lin)
  
  def laser_galvo_callback(self, cmd):
    self.lock.acquire()
    self.power_state[4] = cmd.laser_cmd
    if self.galvo_time == 0.0:
      self.galvo_time = (cmd.galvo_cmd_0 + cmd.galvo_cmd_1) * 0.01
    self.lock.release()
    return SetLaserGalvoResponse()

  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()

      if self.galvo_time > self.dt:
        self.power_state[5] = 1.0
        self.galvo_time -= self.dt
      elif self.galvo_time > 0.0:
        self.power_state[5] = self.galvo_time / self.dt
        self.galvo_time = 0.0
      else:
        self.power_state[5] = 0.0
        self.galvo_time = 0.0
      
      curr_state = self.power_state.copy()

      self.lock.release()

      power = curr_state.dot(self.power_coeff).dot(curr_state)
      self.energy_used += self.dt * power
      
      state_msg = Float32MultiArray()
      state_msg.data = curr_state
      self.power_state_pub.publish(state_msg)
      self.power_pub.publish(power)
      self.energy_used_pub.publish(self.energy_used)

      self.rate.sleep()

if __name__ == '__main__':
  spm = SimPowerModel()
  spm.run()
