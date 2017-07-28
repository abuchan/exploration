#!/usr/bin/python

import sys
import numpy
import matplotlib.pyplot as plt

def save_power_info(power, info, filename='power_info.pdf'):
  fig = plt.figure(1)
  plt.scatter(power[:,0], power[:,1])
  plt.scatter(info[:,0], info[:,1])
  plt.savefig(filename)

if __name__ == '__main__':
  power = numpy.loadtxt(sys.argv[1], delimiter=',')
  info = numpy.loadtxt(sys.argv[2], delimiter=',')
  save_power_info(power, info, sys.argv[3])
