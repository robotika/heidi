Flying autonomous Parrot AR Drone 2 with Python

Lesson 7 - Hovering above oriented roundel

In this lesson you will learn how to use automatically detected tags in
particular "oriented roundel". The sign reminds heliport and in some older
version was even navigation mode FLYING_MODE_HOVER_ON_TOP_OF_ORIENTED_ROUNDEL.
Unfortunately, without notice, this option was removed so in this lesson we
will learn how to do it ourselves.

In your drone box you should already have the sign - if not you can download it
[here](http://ardrone2.parrot.com/media/cms_page_media/38/special_target_1.pdf)
and print it on A4 paper. I recommend to glue it on the box or tape it on the
floor because otherwise it will flew away during takeoff ;-).

The code is still "WORK IN PROGRESS" and what will probably not change is the
used description of detected video tags. In SDK you can find this:

* xc[i], yc[i]: X and Y coordinates of detected tag or oriented roundel #i
inside the picture, with (0; 0) being the top-left corner, and (1000; 1000)
the right-bottom corner regardless the picture resolution or the source
camera.

* width[i], height[i]: Width and height of the detection bounding-box (tag or
oriented roundel #i), when applicable.

* dist[i]: Distance from camera to detected tag or oriented roundel #i in
centimeters, when applicable.

* orientation_angle[i] : Angle of the oriented roundel #i in degrees in the
screen, when applicable.

In the lesson 7 code we try to keep the tag in the middle of the camera image,
i.e. (500,500). Things which we have to consider is the distance in centimeters
(4th parameter), drone angles and its current speed. The goal is to have zero
speed drone.vx, drone.vy and the roundel in the center.

Example of "work in progres" video:
<center>
<iframe width="640" height="360" src="https://www.youtube.com/embed/bGds3axS6xQ?feature=player_detailpage" frameborder="0" allowfullscreen></iframe>
</center>

