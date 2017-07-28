#!/usr/bin/python

import sys
import numpy

def points_to_info(points):
  return points

def info_to_csv(info, filename='info.csv'):
  numpy.savetxt(filename, info, delimiter=',',
    header='time, info')

if __name__ == '__main__':
  points = numpy.loadtxt(sys.argv[1],delimiter=',')
  info = points_to_info(points)
  info_to_csv(info, sys.argv[2])
