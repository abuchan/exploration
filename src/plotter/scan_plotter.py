import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import shapely.geometry as sg
from shapely import affinity
import descartes as dct
import numpy

PI2 = numpy.pi/2.0
PI3 = numpy.pi/3.0

ROBOT_R = 0.15
FOV_R = 2.5
FOV_ANGLE = PI2

MAP_TRAJECTORY = [
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
  ],[
    [2.0, 0.0,  0.0, 1.0],
    [3.0, 2.0, -PI2, 1.0],
  ],[
    [4.0, 0.0,  0.0, 0.0],
    [3.0, 2.0, -PI2, 0.0],
  ],[
    [4.0, 0.0,  PI2, 0.0],
    [3.0, 2.0, -PI2, 0.0],
  ],[
    [4.0, 0.0,  PI2, 1.0],
    [3.0, 2.0,  0.0, 1.0],
  ],[
    [4.0, 0.0,  PI2, 0.0],
    [5.0, 2.0,  0.0, 0.0],
  ],[
    [4.0, 0.0,  PI2, 0.0],
    [5.0, 2.0, -PI2, 0.0],
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
      
MAP_TRAJECTORY_DICT = list_to_dict_traj(MAP_TRAJECTORY)

from matplotlib.ticker import NullFormatter

def plot_trajectory(traj=MAP_TRAJECTORY_DICT, ax=None, fov_angle=FOV_ANGLE):
  if traj is None:
    traj = []

  if ax is None:
    ax = plt.gca()

  scan_geoms = []

  n_r = 3
  n_c = (len(traj)+2)/n_r
  
  next_traj = traj[1:] + [traj[-1]]

  gs1 = gridspec.GridSpec(n_r, n_c)
  gs1.update(wspace=0.02, hspace=0.05)

  for c in range(n_c):
    for r in range(n_r):
      #subax = plt.subplot(n_r, n_c, n_c*r+c+1)
      subax = plt.subplot(gs1[n_c*r+c])
      scan_geoms.extend(plot_scene(
        traj[n_r*c+r]['poses'], next_traj[n_r*c+r]['poses'], scan_geoms, fov_angle=fov_angle
      ))
      #plt.title('$t_{%d}$' % (n_r*c+r))
      if r % 3 == 0:
        action = ': scan, move'
      elif c == n_c-1 and r == n_r-1:
        action = ''
      else:
        action = ': turn'
      subax.text(-0.75,3.25,'t$_{%d}$%s' % (n_r*c+r,action), fontsize=20)
      subax.tick_params(axis='both', which='major', labelsize=15)
      if r % 3 != 2:
        subax.xaxis.set_major_formatter(NullFormatter())
      else:
        subax.set_xlabel('x',fontsize=18)

      if c != 0:
        subax.yaxis.set_major_formatter(NullFormatter())
      else:
        subax.set_ylabel('y',fontsize=18)

def plot_scene(poses=None, next_poses=None, scan_geoms=None, ax=None, fov_angle=FOV_ANGLE):
  if poses is None:
    poses = []

  if next_poses is None:
    next_poses = poses

  if ax is None:
    ax = plt.gca()

  if scan_geoms is None:
    scan_geoms = []

  for scan_geom in scan_geoms:
    ax.add_patch(dct.PolygonPatch(scan_geom, fc='lightgreen', ec=None))

  fov_geoms = [get_fov_geom(fov_angle=fov_angle,**pose) for pose in poses]
  for fov_geom in fov_geoms:
    ax.add_patch(dct.PolygonPatch(fov_geom, fc='lightblue', alpha=0.5))
  
  robot_geoms = [get_robot_geom(**pose) for pose in poses]
  for robot_geom in robot_geoms:
    ax.add_patch(dct.PolygonPatch(robot_geom, fc='yellow'))
  
  # Robot direction arrows
  for i in range(len(poses)):
    pose = poses[i]
    next_pose = next_poses[i]
    
    # Calculate motion delta
    delta = [next_pose[c] - pose[c] for c in ['x','y','theta']]
    
    # Motion arrows
    if abs(delta[0]) > 0.0:
      ax.annotate(
        '',
        (next_pose['x'],next_pose['y']),
        (pose['x']+ROBOT_R,pose['y']),
        size=20,
        arrowprops = dict(arrowstyle="simple", fc="r", ec="none")
      )
    elif abs(delta[2]) > 0.0:
      delta_deg = delta[2]*180/numpy.pi
      theta_deg = pose['theta']*180/numpy.pi
      next_deg = next_pose['theta']*180/numpy.pi
      ax.annotate(
        '',
        (pose['x']+0.8*numpy.cos(next_pose['theta']),pose['y']+0.8*numpy.sin(next_pose['theta'])),
        (pose['x']+0.8*numpy.cos(pose['theta']),pose['y']+0.8*numpy.sin(pose['theta'])),
        size=20,
        arrowprops = dict(
          arrowstyle="simple", 
          fc="r", 
          ec="none",
          connectionstyle="angle3,angleA=%f,angleB=%f" % (theta_deg+delta_deg,next_deg+delta_deg)
        )
      )

    # Robot orientation arrows
    ax.annotate(
      '',
      (pose['x']+0.8*numpy.cos(pose['theta']),pose['y']+0.8*numpy.sin(pose['theta'])),
      (pose['x'],pose['y']), 
      arrowprops=dict(arrowstyle="->")
    )
    ax.text(pose['x']-0.8, pose['y']-0.15, 'R%d'%i, fontsize=15)


  ax.grid(True)
  ax.set_xlim(-0.9, 6.9)
  ax.set_ylim(-1.9, 3.9)
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
