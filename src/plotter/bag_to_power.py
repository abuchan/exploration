#!/usr/bin/python

import sys
import numpy

def bag_to_power(filename='power.bag'):
  return numpy.random.rand(100,2)

def power_to_csv(power, filename='info.csv'):
  numpy.savetxt(filename, power, delimiter=',',
    header='time, power')

if __name__ == '__main__':
  power = bag_to_power(sys.argv[1])
  power_to_csv(power, sys.argv[2])
