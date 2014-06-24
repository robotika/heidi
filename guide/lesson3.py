"""
  Lesson 3 - logging examples
"""
from ardrone2 import ARDrone2, ManualControlException

def testLesson3( drone ):
    try:
        drone.takeoff()
        drone.hover(10.0)
    except ManualControlException, e:
        print "ManualControlException"
    drone.land()
    print "Battery", drone.battery



if __name__ == "__main__":
    import launcher
    import sys
    launcher.launch( sys.argv, ARDrone2, testLesson3 )


# vim: expandtab sw=4 ts=4

