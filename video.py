#!/usr/bin/python
"""
  Extract MPEG-2 video from AR Drone 2.0 stream
  usage:
       ./video.py <logged stream> <output directory> [<startIndex>]
"""
import sys
import os
import struct
import socket

VIDEO_SOCKET_TIMEOUT=1.0

def nextCmd( q ):
  try:
    return q.get(block=False)
  except:
    return None

def logVideoStream( hostPortPair, filename, queueCmd, flushWholeVideo=False ):
  "wait for termination command and otherwise log data"
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
  s.connect( hostPortPair )
  s.settimeout( VIDEO_SOCKET_TIMEOUT )
  f = open( filename, "wb" )
  cmd = None
  while cmd == None:
    try:
      data = s.recv(10240)
      f.write(data)
      f.flush()
    except socket.timeout:
      print "Video filename TIMEOUT"
    cmd = nextCmd( queueCmd )
  if flushWholeVideo:
    while True:
      try:
        data = s.recv(10240)
        f.write(data)
        f.flush()
      except socket.timeout:
        print "REC Video Completed (TIMEOUT)"
        break
  f.close()
  s.close()

def convertVideo( filename, outDir, frameIndex = 0 ):
  fin = open( filename, "rb")
  save = False
  data = fin.read(4)
  while len(data) > 0:
    assert data == "PaVE" # PaVE (Parrot Video Encapsulation)
    print "version", ord(fin.read(1))
    print "codec", ord(fin.read(1))
    headerSize = struct.unpack_from("H", fin.read(2))[0]
    print "header size", headerSize
    payloadSize = struct.unpack_from("I", fin.read(4))[0]
    print "payload size", payloadSize
    buf = fin.read(headerSize - 12)
    print "BUF", len(buf)
    arr = struct.unpack_from("HHHHIIBBBBIIHBBBBBBI",buf) # resolution, index, chunk, type, (ignored III reserved)
    print "ARR", arr
    print "Frame number", arr[4]
    if arr[8]==1:
      save = True
#    print "chunk", struct.unpack_from("",buf[16:])
    payload = fin.read(payloadSize)
    print "Payload Data:", [ord(c) for c in payload[:8]]
    if save:
      fout = open(outDir+os.sep+"frame%04d.bin" % frameIndex, "wb")
      fout.write(payload)
      fout.close()
      frameIndex += 1
    data = fin.read(4)


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(2)
  frameIndex = 0
  if len(sys.argv) > 3:
    frameIndex = int(sys.argv[3])
  convertVideo( sys.argv[1], sys.argv[2], frameIndex=frameIndex )

