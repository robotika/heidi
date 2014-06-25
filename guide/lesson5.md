Flying autonomous Parrot AR Drone 2 with Python

Lesson 5 - Fly at given height

How was flying in previous lesson? Did you experiment with longer times? Did
you crash? If yes then you are ready for next lesson - it is time to control
drone height/altitude and forward speed.

AR Drone 2 has several data sources to estimate current height. There is bottom
looking camera, sonar and pressure sensor. Also speeds of rotors and
accelerometers are probably taken into account. If you do not want to worry
about details (95% of users) you can simply use Z coordinate (i.e.
"drone.coord[2]"). This works fine, but you should be aware that it is not
perfect. It is just estimate integrated over your flight time. Also it is like
"global altitude" with reference to your start point. What would you expect if
you fly on uneven terrain?

[Here](http://www.youtube.com/watch?v=qhmc--pTRJM) you can see my quadcopter
Heidi crashing into ground. Our garden is not absolutely flat and if you fly at
1m at some point you will hit it.

Another memorable experience is from [Air Race
contest](http://www.youtube.com/watch?v=brrHXWxjO9k) where Isabelle, our second
drone, was supposed to autonomously fly 8 figures. It was OK at the beginning
but started to drop height over time (you can move the video to 6:20 and see
yourself). Why? because we blindly trusted "drone.coord[2]". The drone does not
know that it still the same floor. Once a while somebody opened the door and
due to small breeze the pressure changed. The error grows.

So what you can do? Well, then you have to take care of details what is in the
reality integrated. You know more about the environment. You know that the
floor is flat or that you actually want to rather fly at 1.5m _from the ground_
and not at given level.  You can access directly row readings of sonar and
camera height estimation.  They are available at "drone.altitudeData". The data
do not have to be always available - the drone does not have to send them with
200Hz frequency. That is why you see " if drone.altitudeData != None:" in the
code. Also these are raw values, in millimeters ... we may change this in some
later ARDrone2 class revision, because over time we see that these data are
really useful.

If you fly in reasonable conditions the estimation from sonar and from camera
are very close. But as soon as you start to fly above terrain with texture
(field with maize, for example) and you will feel the wind breeze you can be
surprised :-). That is explanation for
    
    if abs(altSonar-altVision) > 0.5:
        print altSonar, altVision
        altitude = max( altSonar, altVision ) # sonar is 0.0 sometimes (no ECHO)

Also sometimes you do not get echo from sonar and the reading will be 0.

If you flew more than 5 seconds in previous lesson you probably found out how
much is the drone speeding up. It is not safe and you do not want it fast
anyway.  This time we have simple forward speed control:

    sx = max( 0, min( drone.speed, desiredSpeed - drone.vx ))

TODO refactoring - drone.vx and desiredSpeed are both in metric units (meters
per second), but sx and drone.speed are strange Parrot fractions of maximal
allowed tilt. This code works, but the units are not matching ...


Homework

Try to fly up and down the stairs.

