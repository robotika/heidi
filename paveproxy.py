#!/usr/bin/python
"""
  Socket for filtering PaVE (Parrot Video Encapsulation) packets
       ./paveproxy.py <input file>
"""
import sys
import struct
import socket

from threading import Thread,Event,Lock
from pave import PaVE

HOST = 'localhost'    # access URL for cv2.VideoCapture
PORT = 50007
BUFFER_SIZE = 1024

class PaVEProxy( Thread ):
  def __init__( self, readCbk ):
    Thread.__init__( self )
    self.setDaemon(True)
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socket, self.addr = None, None
    self.readCbk = readCbk
    self.pave = PaVE()
    self.start()

  def run( self ):
    "wait only for the first client - delayed binding"
    self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.serverSocket.bind((HOST, PORT))
    self.serverSocket.listen(1)
    self.socket, self.addr = self.serverSocket.accept() 
    while self.shouldIRun.isSet():
      self.pave.append( self.readCbk( BUFFER_SIZE ) )
      header,payload = self.pave.extract()
      if payload:
        self.socket.send( payload )

  def term( self ):
    self.shouldIRun.clear()
    if self.socket==None:
      print "KILLING run()"
      tmpSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      tmpSocket.connect((HOST, PORT))
      tmpSocket.close()
      print "DONE"


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)

