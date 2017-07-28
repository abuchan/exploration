#!/usr/bin/python

import sys
import numpy
from digifab import *

def polymesh_volume(polymesh):
  stl_data = {'points':polymesh.points, 'faces':polymesh.indices}
  data = numpy.zeros(len(stl_data['faces']), dtype=stl.mesh.Mesh.dtype)
  data['vectors'] = [
    stl_data['points'][idxs,0:3] for idxs in stl_data['faces']
  ]
  m = stl.mesh.Mesh(data, remove_empty_areas=False)
  return m.get_mass_properties()[0]

Z_AXIS = numpy.array([0,0,1])

# 1-robot-diameter cylinder extruded along scan path
def scan_cylinder(point, camera, r_dia = 0.1):
  point = numpy.array(point)/r_dia
  camera = numpy.array(camera)/r_dia
  vector = point - camera
  v_len = vector.dot(vector)**0.5
  axis = numpy.cross(Z_AXIS, vector/v_len)
  angle = numpy.arcsin(axis.dot(axis)**0.5)
  H = quaternion_matrix(quaternion_about_axis(angle, axis))
  H[:3,3] = camera
  H_list = [list(r) for r in H]
  gen = solid.multmatrix(H_list)(
    solid.cylinder(r=0.5, h=v_len)
  )
  geom = PolyMesh(generator=gen)
  return geom
  
def recover_partial_info(points, folder='meshes', end_index=-1):
  files = sorted(glob.glob('%s/*.stl' % folder))[:end_index]
  last_file = files[-1]
  index = int(last_file.split('_')[-1].split('.')[0])
  scans = [PolyMesh(filename=f) for f in files]
  info = [[points[i,0],polymesh_volume(scans[i])] for i in range(len(scans))]
  return scans[-1], info, index

def points_to_info(points, starting_point = None):
  
  scan = PolyMesh()
  index = 0
  
  if starting_point is None:
    info = []
    start_index = 0
  else:
    _, info, start_index = starting_point

  for point in points:
    time = point[0]
    pt = point[1:4]
    c0 = point[4:7]
    c1 = point[7:10]
    scan += (scan_cylinder(pt, c0) + scan_cylinder(pt, c1))
    
    print 'Info %d of %d' % (index, len(points))
    if index >= start_index:
      print 'saving'
      scan.save('meshes/t_%04d.stl' % index)
      info.append([time, polymesh_volume(scan)])
    
    index += 1
  
  return numpy.array(info)

def info_to_csv(info, filename='info.csv'):
  numpy.savetxt(filename, info, delimiter=',',
    header='time, info')

if __name__ == '__main__':
  points = numpy.loadtxt(sys.argv[1],delimiter=',')
  info = points_to_info(points)
  info_to_csv(info, sys.argv[2])
