#!/usr/bin/python

import sys
import numpy
from rosbag.bag import Bag

def bag_to_power(filename='power.bag'):
  bagfile = Bag(filename)

  energies = {}

  power = []
  for topic, msg, timestamp in bagfile.read_messages():
    if topic.find('energy') != -1:
      energies[topic] = msg.data
      power.append([timestamp.to_sec(), sum(energies.values())])

  bagfile.close()
    
  return numpy.array(power)

def power_to_csv(power, filename='info.csv'):
  numpy.savetxt(filename, power, delimiter=',',
    header='time, power')

if __name__ == '__main__':
  power = bag_to_power(sys.argv[1])
  power_to_csv(power, sys.argv[2])
