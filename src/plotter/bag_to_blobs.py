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

H_w_r0_static = numpy.eye(4)
H_w_r1_static = H_r_c.dot(H_c0_r1)

Z3 = numpy.zeros(3).reshape(-1,1)
I3 = numpy.eye(3)

#f_len = 554.2562397718481
f_len = 457.0073649484105
n_x = 640.0
n_y = 480.0

def pair_to_point(blob_pair, H_w_r0=H_w_r0_static, H_w_r1=H_w_r1_static):
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

  H_w_c0 = H_w_r0.dot(H_r_c)
  H_w_c1 = H_w_r1.dot(H_r_c)

  c0_w = H_w_c0[:3,3]
  c1_w = H_w_c1[:3,3]

  y = numpy.hstack([c0_w,c1_w])
  
  A_inv = numpy.linalg.inv(A.T.dot(A)).dot(A.T)
  
  x = A_inv.dot(y)

  return x[:3], c0_w, c1_w

def plot_points(pts):
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  ax.scatter(pts[:,0], pts[:,1], pts[:,2])
  ax.set_xlabel('X(m)')
  ax.set_ylabel('Y(m)')
  ax.set_zlabel('Z(m)')
  plt.show()

def pose_to_matrix(pose):
  pos, ori = pose[3:]
  H = quaternion_matrix(ori)
  H[:3,3] = pos
  return H

def blobs_poses_to_points(blobs, poses):
  merged = blobs + poses
  merged.sort()
  
  last_H_w_r0 = None
  last_H_w_r1 = None
  
  points = []

  for item in merged:
    
    # pose entry
    if type(item[1]) is str and item[1] == 'world':
      if item[2] == '/robot_00_mocap':
        last_H_w_r0 = item
      elif item[2] == '/robot_01_mocap':
        last_H_w_r1 = item

    # blob entry
    elif last_H_w_r0 is not None and last_H_w_r1 is not None:
      time, pair = item[0], item[1:]
      H0 = pose_to_matrix(last_H_w_r0)
      H1 = pose_to_matrix(last_H_w_r1)
      pt, c0, c1 = pair_to_point(pair, H0, H1)
      points.append([time] + list(pt) + list(c0) + list(c1))

  return numpy.array(points)

def points_to_csv(points, filename='points.csv'):
  numpy.savetxt(filename, points, delimiter=',',
    header='time,p_x,p_y,p_z,c0_x,c0_y,c0_z,c1_x,c1_y,c1_z')

def bag_to_blobs(filename):
  bagfile = Bag(filename)

  blobs = []
  b_time = None
  curr_blobs = [None, None]
  poses = []
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
    
    elif topic == '/tf':
      for tf_msg in msg.transforms:
        p_time = tf_msg.header.stamp.to_sec()
        base_frame = tf_msg.header.frame_id
        child_frame = tf_msg.child_frame_id
        pos = tf_msg.transform.translation
        pos = [pos.x, pos.y, pos.z]
        ori = tf_msg.transform.rotation
        ori = [ori.x, ori.y, ori.z, ori.w]
        poses.append([p_time, base_frame, child_frame, pos, ori])

  bagfile.close()

  return blobs, poses

if __name__ == '__main__':
  blobs, poses = bag_to_blobs(sys.argv[1])
  points = blobs_poses_to_points(blobs, poses)
  points_to_csv(points,sys.argv[2])
