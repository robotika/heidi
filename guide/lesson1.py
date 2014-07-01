"""
  Lesson 1 - before you takeoff ...
"""
import sys
sys.path.append('..') # access to drone source without installation
from ardrone2 import ARDrone2

drone = ARDrone2()

drone.wait(10.0)      # active waiting for 10 seconds
print drone.battery
print drone.coord     # X, Y, Z coordinate

