#!/usr/bin/python

import sys
import numpy
import matplotlib.pyplot as plt

def interp_power(power, info, clock):
  power_clock_interp = numpy.interp(clock[:,0],power[:,0],power[:,1])
  power_shift = numpy.vstack([clock[:,1],power_clock_interp]).T
  power_info_interp = numpy.interp(info[:,0],power_shift[:,0],power_shift[:,1])
  return power_info_interp

def save_power_info(power, info, clock, filename='power_info.pdf'):
  fig = plt.figure(1)
  power_info_interp = interp_power(power, info, clock)
  plt.scatter(power_info_interp, info[:,1], c='k', marker='.')
  plt.xlabel('Energy (J)')
  plt.ylabel('Info (Robot Volumes)')
  plt.grid(True)
  plt.savefig(filename)

if __name__ == '__main__':
  power = numpy.loadtxt(sys.argv[1], delimiter=',')
  info = numpy.loadtxt(sys.argv[2], delimiter=',')
  clock = numpy.loadtxt(sys.argv[3], delimiter=',')
  save_power_info(power, info, clock, sys.argv[4])
