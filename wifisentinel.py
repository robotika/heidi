#!/usr/bin/python
"""
  WIFI network protector during flight
  usage:
       ./protector.py <internal program with args>
"""
import subprocess
import time
import sys

DRONE_IP_LIST = ['192.168.1.2', '192.168.1.3', '192.168.1.4']

def myWiFiIP():
  # TODO linux
  ret = None
  p = subprocess.Popen( "ipconfig", stdout=subprocess.PIPE )
  for line in p.stdout.readlines():
    if "IPv4 Address" in line:
      ret = line.strip().split()[-1]
  return ret

while True:
  ip = myWiFiIP()
  while ip not in DRONE_IP_LIST:
    print ip
    time.sleep(10)
    ip = myWiFiIP()
  print "CONNECTED TO", ip
  if subprocess.call( sys.argv, shell=True ) == 0:
    break
  print "RECOVERY"

