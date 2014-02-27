#!/usr/bin/python
"""
  Universal launcher for various competitions/drones
  usage:
       ./<competition>.py <task|reply> [<reply log> [F]]
"""
import datetime
import viewlog
import ardrone2

# only as wrapper for kbhit
import sys
if sys.platform == 'linux2':
  import pygame


def linuxKbHit():
  "workaround for msvc.kbhit, requires pygame"
  events = pygame.event.get()
  for event in events:
    if event.type == pygame.KEYDOWN:
      return True
  return False


def launch(cmd_args, robotFactory, task, configFn = None, canArgs={}):
  '''
  Launches a robot to the given file.

  Parameters:
  cmd_args     ... List of parameters in the "configFile [--file logfile [F|FF]]" form.
  robotFactory ... Function (or class) which can be called with the following named
                   parameters and creates an instance of the robot:
		   robot = robotFactory(can=..., replyLog=...)
  task         ... Factory which called as "t = task(robot, configFileName, verbose=...)"
                   creates a runable instance. Ie. "t()" runs the show. 
  configFn     ... A method called in the robot's preconfigration mode.
  canArgs      ... Named arguments passed down to the CAN.
  '''
  if len(cmd_args) < 2:
    print __doc__
#    sys.exit(2)
    return

  # TODO unified launcher, similar to Eduro
  if len(cmd_args) > 3 and cmd_args[3] == 'F':
    ardrone2.g_checkAssert = False
  replayLog = None
  metaLog = None
  console = None
  if len(cmd_args) > 2:
    if "meta" in cmd_args[2]:
      metaLog = open(cmd_args[2])
      for line in metaLog:
        if line.startswith("navdata:"):
          replayLog = line.split()[1].strip()
          break
    else:
      replayLog=cmd_args[2]
    viewlog.viewLogFile = open( "view.log", "w" )
    viewlog.dumpSharpsGeometry( [(0.18, 0.0, 0.0)] ) # front sonar
  else: # create new metaLog
    metaLog = open( datetime.datetime.now().strftime("logs/meta_%y%m%d_%H%M%S.log"), "w" )
    metaLog.write( str(cmd_args) + "\n" )    
    metaLog.flush()
    if sys.platform == 'linux2':
      pygame.init()
      screen = pygame.display.set_mode((100,100))
      console = linuxKbHit

  if cmd_args[1] == "replay":
    for replayLog in cmd_args[2:]:
      drone = ARDrone2( replayLog, skipConfigure=True )
      try:
        while True:
          drone.update()
          if drone.altitudeData:
            print "%d\t%.3f\t" % (drone.ctrlState, drone.coord[2]) + "\t".join([str(x) for x in drone.altitudeData])
      except EOFError:
        pass
  else:
    task( robotFactory( replayLog=replayLog, metaLog=metaLog, console=console ) )

