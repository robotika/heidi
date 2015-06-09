"""
  Lesson 7 - hover above oriented roundel sign
"""
import sys
import os
import inspect
import math

ARDRONE2_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if ARDRONE2_ROOT not in sys.path:
    sys.path.insert(0, ARDRONE2_ROOT) # access to drone source without installation

from ardrone2 import ARDrone2, ManualControlException, normalizeAnglePIPI

MAX_ALTITUDE = 3.0

def getXYZAGoalCmd( drone, goal, goalHeading=None, maxSpeed=0.3 ):
    frac = 0.3
    dxWorld = goal[0] - drone.coord[0]
    dyWorld = goal[1] - drone.coord[1]
    c = math.cos(drone.heading)
    s = math.sin(drone.heading)
    dx = c*dxWorld + s*dyWorld   # inverse rotation
    dy = -s*dxWorld + c*dyWorld
    sx = max( -maxSpeed, min( maxSpeed, frac*(dx - drone.vx) ))
    sy = max( -maxSpeed, min( maxSpeed, frac*(dy - drone.vy) ))
    sa = 0.0
    if goalHeading:
        sa = max( -maxSpeed, min( maxSpeed, frac*(normalizeAnglePIPI(goalHeading - drone.heading)) ))
    return sx, sy, 0.0, sa
    


def hoverAboveRoundel( drone, timeout=6000.0 ):
    startTime = drone.time
    maxSpeed = 0.1
    maxSpeedUpDown = 0.3
    frac = 0.1
    scaleX, scaleY = 0.38, 0.38 # camera FOW at 1m above the ground
    detectedCount = 0
    goal = (0.0, 0.0) #None
    goalHeading = None
    while drone.time - startTime < timeout:
        sx, sy, sz, sa = 0.0, 0.0, 0.0, 0.0
        if drone.visionTag:
            # xc[i], yc[i]: X and Y coordinates of detected tag or oriented roundel #i inside the picture,
            # with (0; 0) being the top-left corner, and (1000; 1000) the right-bottom corner regardless
            # the picture resolution or the source camera
            x, y = drone.visionTag[0][:2]
            dist = drone.visionTag[0][4]/100.
            angle = math.radians(drone.visionTag[0][5] - 270) # we use circle for the body and line for the tail -> 270deg offset
            detectedCount = min(100, detectedCount + 1)
            # TODO verify signs and scale
            dx = dist*(scaleY*(500-y)/500.0 + drone.angleFB)
            dy = dist*(scaleX*(500-x)/500.0) # - drone.angleLR)
            c = math.cos(drone.heading)
            s = math.sin(drone.heading)
            goal = (drone.coord[0] + c*dx - s*dy, 
                    drone.coord[1] + s*dx + c*dy )
            goalHeading = normalizeAnglePIPI( drone.heading + angle ) # TODO check sign
#            print drone.visionTag, "%.2f %.2f %.1f %.1f" % (dx, dy, math.degrees(drone.angleFB), math.degrees(drone.angleLR))
        else:
            detectedCount = max(0, detectedCount - 1)

        if goal:
            sx,sy,sz,sa = getXYZAGoalCmd( drone, goal, goalHeading=goalHeading )

        sz = 0.0
        if drone.altitudeData != None:
            altVision = drone.altitudeData[0]/1000.0
            altSonar = drone.altitudeData[3]/1000.0
            if drone.altitudeData[3] == 0:
                # no sonar echo
                minAlt = altVision
            else:
                minAlt = min(altSonar, altVision)

            if minAlt > MAX_ALTITUDE:
                # too high to reliably detect anything
                sz = -maxSpeedUpDown

#            print altSonar, altVision,
            if detectedCount == 0:
                # try to move up
                if max(altSonar, altVision) < 2.0:
                    sz = maxSpeedUpDown
            elif detectedCount == 100:
                if minAlt > 1.0:
                    sz = -maxSpeedUpDown
#        print "\t%.2f %.2f %.2f %.2f" % (sx, sy, sz, sa)
        drone.moveXYZA( sx, sy, sz, sa )
    drone.hover(0.1) # stop motion

def testLesson7( drone ):
    try:
        drone.startVideo( record=True, highResolution=False )
        drone.setVideoChannel( front=False )
#        drone.startVideo( record=True, highResolution=True )
#        drone.setVideoChannel( front=True )
        drone.takeoff()
        hoverAboveRoundel( drone )
    except ManualControlException, e:
        print "ManualControlException"
    if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion
        drone.hover(0.1)
    drone.land()
    drone.stopVideo()
    print "Battery", drone.battery


if __name__ == "__main__":
    import launcher
    launcher.launch( sys.argv, ARDrone2, testLesson7 )


# vim: expandtab sw=4 ts=4

