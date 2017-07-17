#!/usr/bin/python

import sys
from rosbag.bag import Bag

import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from tf.transformations import *

c0_r1_pos = [-0.502606312708, 0.148741325874, 1.00303366614]
c0_r1_ori = [0.711964297011, 0.00385394381228, 0.00238965009409, 0.702201022835]
H_c0_r1 = quaternion_matrix(c0_r1_ori)
H_c0_r1[:3,3] = c0_r1_pos

r_c_pos = [0.0, 0.0, 0.15]
r_c_ori = [-0.5, 0.5, -0.5, 0.5]
H_r_c = quaternion_matrix(r_c_ori)
H_r_c[:3,3] = r_c_pos

H_w_r0 = numpy.eye(4)
H_w_r1 = H_r_c.dot(H_c0_r1)

H_w_c0 = H_w_r0.dot(H_r_c)
H_w_c1 = H_w_r1.dot(H_r_c)

c0_w = H_w_c0[:3,3]
c1_w = H_w_c1[:3,3]

y = numpy.hstack([c0_w,c1_w])
Z3 = numpy.zeros(3).reshape(-1,1)
I3 = numpy.eye(3)

f_len = 554.2562397718481
n_x = 640.0
n_y = 480.0

def pair_to_point(blob_pair):
  x0, y0, x1, y1 = blob_pair
  v0_c = numpy.array([x0-n_x/2, y0-n_y/2, f_len, 0.0])
  v1_c = numpy.array([x1-n_x/2, y1-n_y/2, f_len, 0.0])
  v0_c = v0_c / (v0_c.dot(v0_c)**0.5)
  v1_c = v1_c / (v1_c.dot(v1_c)**0.5)
  v0_w = H_w_r0.dot(H_r_c.dot(v0_c))[:3].reshape(-1,1)
  v1_w = H_w_r1.dot(H_r_c.dot(v1_c))[:3].reshape(-1,1)
  
  A = numpy.vstack([
    numpy.hstack([I3, -v0_w, Z3]),
    numpy.hstack([I3, Z3, -v1_w])
  ])
  A_inv = numpy.linalg.inv(A.T.dot(A)).dot(A.T)
  x = A_inv.dot(y)

  return x[:3]

def plot_points(pts):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(pts[:,0], pts[:,1], pts[:,2])
  ax.set_xlabel('X(m)')
  ax.set_ylabel('Y(m)')
  ax.set_zlabel('Z(m)')
  plt.show()

def bag_to_blobs(filename):
  bagfile = Bag(filename)

  blobs = []
  b_time = None
  curr_blobs = [None, None]

  start_time = bagfile.get_start_time()
  
  for topic, msg, timestamp in bagfile.read_messages():
    if topic[-9:] == 'blob_list':
      if len(msg.blobs) == 1:
        m_time = msg.header.stamp.to_sec()
        r_idx = int(topic[-20])
        blob = [msg.blobs[0].x, msg.blobs[0].y]
        #blobs.append([m_time,r_idx] + blob)
        
        if b_time is None:
          b_time = m_time
          curr_blobs[r_idx] = blob
        
        elif abs(b_time-m_time) < 0.001:
          curr_blobs[r_idx] = blob
          blobs.append([b_time] + curr_blobs[0] + curr_blobs[1])
          b_time = None
          curr_blobs = [None, None]

        else:
          b_time = m_time
          curr_blobs = [None, None]
          curr_blobs[r_idx] = blob

  bagfile.close()
  return blobs

if __name__ == '__main__':
  blobs = bag_to_blobs(sys.argv[1])
