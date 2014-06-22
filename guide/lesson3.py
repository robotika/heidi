"""
  Lesson 2 - manual emergency stop
"""
from ardrone2 import ARDrone2, ManualControlException

def testLesson3( drone ):
    try:
        drone.takeoff()
        drone.hover(10.0)
    except ManualControlException, e:
        print "ManualControlException"
    drone.land()



if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ARDrone2, testLesson3 )


# vim: expandtab sw=4 ts=4

