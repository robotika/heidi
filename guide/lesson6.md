Flying autonomous Parrot AR Drone 2 with Python

Lesson 6 - Record video of surrounding

In this lesson you will learn how to record video and turn 360 degrees. You
already know how to turn (last parameter in "drone.moveXYZA()" function) so new
is how to start video recording and how to limit the rotation.

AR Drone 2 has two cameras - one in the front and second pointing down. The
first one is much better and it is meant for recording interesting videos. The
down pointing camera is primarily for internal use, when drone estimates motion
and height for low altitudes.

There are two *video channels*: one for video recording and one for quick video
preview (with lower quality). If you are interested you may check the source
codes - basically it opens another TCP connection and reads video data.

    drone.startVideo( record=True, highResolution=True )
    drone.setVideoChannel( front=True )

With these two lines to start recording video and select front camera
(default). When the drone lands you should stop the video via call

    drone.stopVideo()

And that is all. Feel free to experiment with resolutions and front/bottom
camera selection. Note, that in current firmware (2.4.x) it is not possible to
select one camera for preview channel and second camera for recording.

And what about the 360deg scanning?! Well, you are right! What is the simplest
solution for that task? We will implement "version 0" and that is only with
time now. Are you disappointed? Try it first!

There are several important details you should learn from Version 0:
- drone does not stay in place
- even the height is varying so you maybe had to cut the fly with emergency stop
- heading is bit random (you can add print drone.heading and replay your 
attempt from log file, maybe draw a graph)

I hope that now you are a little bit convinced that even the simplest solution
was not useless. This corresponds to [eXtreme
programming](http://en.wikipedia.org/wiki/Extreme_programming) practises - test
first. You get real data and some idea what is going on. On your log files
you can then repeat it many times and write version 1 (homework) which will
keep given hight (say 1.5 meter above the ground), stay in place (you can use
drone.coord and reference point when you entered the scan function), and rotate
360 degrees.

Note, that heading is a result of data fusion (at least compass + gyros). It
takes some time before the estimate converge to some stable value ... so you
may want first to go to 1.5 meter height and then start to trust heading
readings. And a small _cherry on the top_ ... the data fusion is so good,
that it took us a month (?) before we realized that compass was broken on
Isabelle1 - there was probably some bad connection so sometimes we got the data sometimes
not. Gyroscopes took over very soon ... tough one. Do you trust your drone
compass? If not you can dump row compass data: "drone.magneto". The whole story
is [here](http://robotika.cz/competitions/robotchallenge/2014/en#140312).


Final note about line

    scanVersion0( drone )

Why is it this way and not

    drone.scanVersion0()

? This is because we are calling function "scanVersion0()" with parameter
"drone" and this function is not part of ARDrone2 class. I would also prefer
second style, but in that case we would need to extend ARDrone2 class and add
there new function.
