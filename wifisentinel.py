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
  ret = None
  if sys.platform == 'linux2':
    p = subprocess.Popen( ["/sbin/ifconfig", "wlan0"], stdout=subprocess.PIPE )
    for line in p.stdout.readlines():
      if "inet adr:" in line: # Czech linux (!)
        ret = line.strip().split()[1][4:] # cut "adr:"
  else:
    p = subprocess.Popen( "ipconfig", stdout=subprocess.PIPE )
    for line in p.stdout.readlines():
      if "IPv4 Address" in line and "192.168.56.1" not in line:
        ret = line.strip().split()[-1]
  return ret

print sys.argv
while True:
  ip = myWiFiIP()
  while ip not in DRONE_IP_LIST:
    print ip
    time.sleep(3)
    ip = myWiFiIP()
  print "CONNECTED TO", ip
  if subprocess.call( sys.argv[1:] ) == 0:
    break
  print "RECOVERY"

