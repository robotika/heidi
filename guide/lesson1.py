"""
  Lesson 1 - basic functions (takeoff, hover and land)
  Beware of missing manual control! Fly at large open places without wind!
"""
from ardrone2 import ARDrone2

drone = ARDrone2()
drone.takeoff()
drone.hover(3.0)
drone.land()

