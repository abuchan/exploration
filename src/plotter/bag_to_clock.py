#!/usr/bin/python

import sys
import numpy
from rosbag.bag import Bag

def bag_to_clock(filename='clock.bag'):
  bagfile = Bag(filename)
  
  clock = []

  for topic, msg, timestamp in bagfile.read_messages():
    if topic == '/clock':
      clock.append([timestamp.to_sec(), msg.clock.to_sec()])

  return numpy.array(clock)

def clock_to_csv(clock, filename='clock.csv'):
  numpy.savetxt(filename, clock, delimiter=',',
    header='wall,sim')

if __name__ == '__main__':
  clock = bag_to_clock(sys.argv[1])
  clock_to_csv(clock, sys.argv[2])
