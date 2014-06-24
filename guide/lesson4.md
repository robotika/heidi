Flying autonomous Parrot AR Drone 2 with Python

Lesson 4 - Fly forward

It is time to fly! (and not only hover on one place) If you did not follow our
recommendation in lesson 1 you should do it now. Find some free space!
Sometimes even the takeoff is a little bit wild and if you have enough space
you get also enough time to hit the emergency stop (keyboard) and land.

Original Parrot API uses a bit strange coordinate system. We are used to ground
vehicles, where XY was enough, so we transform it. X is pointing forward,
Y left and Z up. The drone starts from (0,0,0) and you can read current
position from drone.coord internal variable. Distances are in meters.

There are four dimensions in which you can control the drone: forward/backward,
left/right, up/down, anticlockwise/clockwise. These are also four parameters in
drone.moveXYZA() function. Note, that you control fractions of some
pre-configured values! sx=0.1 is 10% of maximal allowed tilt. If you
increase the time from 2.0s say to 10s be aware the THE DRONE WILL SPEED UP!
Also if you set after that sx to zero, it will still continue to fly in previous
direction! Sounds scary?

One remark about following two lines: 
if drone.ctrlState == 3: # CTRL_FLYING=3 ... i.e. stop the current motion drone.
   hover(0.1) 

The drone is in some "state" and if you hit emergency stop, while flying really
fast in some direction, you first want to stop that movement. And that's what
our old "hover" will do for us.

Homework 1:

Change the code that the drone will land as soon as it is 2 meters from
starting position.

