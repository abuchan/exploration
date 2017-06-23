import matplotlib.pyplot as plt
import shapely.geometry as sg
from shapely import affinity
import descartes as dct
import numpy

PI2 = numpy.pi/2.0
PI3 = numpy.pi/3.0

ROBOT_R = 0.1
FOV_R = 2.5
FOV_ANGLE = 3*PI2

trajectory = [
  [
    [0.0, 0.0,  0.0, 1.0],
    [1.0, 2.0, -PI2, 1.0],
  ],[
    [2.0, 0.0,  0.0, 0.0],
    [1.0, 2.0, -PI2, 0.0],
  ],[
    [2.0, 0.0,  PI2, 0.0],
    [1.0, 2.0, -PI2, 0.0],
  ],[
    [2.0, 0.0,  PI2, 1.0],
    [1.0, 2.0,  0.0, 1.0],
  ],[
    [2.0, 0.0,  PI2, 0.0],
    [3.0, 2.0,  0.0, 0.0],
  ],[
    [2.0, 0.0,  PI2, 0.0],
    [3.0, 2.0, -PI2, 0.0],
  ]
]

def list_to_dict_traj(np_traj):
  dict_traj = []
  for t_i in range(len(np_traj)):
    dict_traj.append({
      'time': 't%d' % t_i, 
      'poses': [
        dict(zip(['x','y','theta','scan'],r_pose)) for r_pose in np_traj[t_i]
       ]
     })
  
  return dict_traj
      
def plot_trajectory(traj=None, ax=None):
  if traj is None:
    traj = []

  if ax is None:
    ax = plt.gca()

  scan_geoms = []

  n_r = 3
  n_c = (len(traj)+2)/n_r
  
  print n_r, n_c

  for c in range(n_c):
    for r in range(n_r):
      print r,c,n_c*r+c,n_r*c+r+1
      plt.subplot(n_r, n_c, n_c*r+c+1)
      scan_geoms.extend(plot_scene(traj[n_r*c+r]['poses'],scan_geoms))
  
def plot_scene(poses=None, scan_geoms=None, ax=None):
  if poses is None:
    poses = []

  if ax is None:
    ax = plt.gca()

  if scan_geoms is None:
    scan_geoms = []

  for scan_geom in scan_geoms:
    ax.add_patch(dct.PolygonPatch(scan_geom, fc='lightgreen', ec=None))

  fov_geoms = [get_fov_geom(**pose) for pose in poses]
  for fov_geom in fov_geoms:
    ax.add_patch(dct.PolygonPatch(fov_geom, fc='lightblue', alpha=0.5))
  
  robot_geoms = [get_robot_geom(**pose) for pose in poses]
  for robot_geom in robot_geoms:
    ax.add_patch(dct.PolygonPatch(robot_geom, fc='yellow'))

  ax.grid(True)
  ax.set_xlim(-1.0, 7.0)
  ax.set_ylim(-2.0, 4.0)
  ax.set_aspect('equal')
  
  new_scan_fovs = [fov_geom for fov_geom, pose in zip(fov_geoms, poses) if pose['scan']]
  if len(new_scan_fovs) > 1:
    new_scan_geom = new_scan_fovs[0]
    for new_scan_fov in new_scan_fovs[1:]:
      new_scan_geom = new_scan_geom.intersection(new_scan_fov)
    return [new_scan_geom]
  else:
    return []

def get_robot_geom(x=0.0, y=0.0, theta=0.0, robot_r=ROBOT_R, **kwargs):
  return sg.Point(x,y).buffer(robot_r)

def get_fov_geom(x=0.0, y=0.0, theta=0.0, fov_r=FOV_R, fov_angle=FOV_ANGLE, **kwargs):
  disk = sg.Point(0.0,0.0).buffer(fov_r)

  mask_x = 1.1*fov_r/numpy.tan(fov_angle/2)

  mask_points = [
    (mask_x, -1.1*fov_r), (0.0, 0.0), (mask_x, 1.1*fov_r)
  ]
  if mask_x > -1.1*fov_r:
    mask_points.extend([(-1.1*fov_r, 1.1*fov_r), (-1.1*fov_r, -1.1*fov_r)])

  mask = sg.Polygon(mask_points)
  
  fov = disk.difference(mask)
  fov_r = affinity.rotate(fov,theta,(0.0,0.0),use_radians=True)
  fov_t = affinity.translate(fov_r,x,y)

  return fov_t
