#!/usr/bin/python
"""
  re-pack PaVE (Parrot Video Encapsulation) packets
       ./pave.py <input file> <packet size>
"""
import sys
import struct

class PaVE:
  def __init__( self ):
    self.buf = ""

  def append( self, packet ):
    self.buf += packet


  def extract( self ):
    "return single packet (header, payload)"
    if not self.buf.startswith("PaVE"):
      if "PaVE" in self.buf:
        self.buf = self.buf[ self.buf.index("PaVE") : ]

    if len(self.buf) < 4+1+1+2+4:
      # at least struct of version and header and payload sizes must be ready
      return "",""
    
    if not self.buf.startswith( "PaVE" ):
      return "",""

    version, codec, headerSize, payloadSize = struct.unpack_from("BBHI", self.buf, 4 )
    if len(self.buf) < headerSize + payloadSize:
      return "",""

    ret = self.buf[:headerSize], self.buf[headerSize:headerSize+payloadSize]
    self.buf = self.buf[headerSize + payloadSize : ]
    return ret

def isIFrame( header ):
  "return True if I-Frame"
  return struct.unpack_from("B", header, 30)[0] == 1

def frameNumber( header ):
  return struct.unpack_from("I", header, 20)[0]

def timestamp( header ):
  return struct.unpack_from("I", header, 24)[0]


if __name__ == "__main__":
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(2)

  filename, size = sys.argv[1], int(sys.argv[2])
  merger = PaVE()
  f = open( filename, "rb" )
  tmp = f.read( size )
  while tmp != "":
    merger.append( tmp )
    header, payload = merger.extract()
    while payload != "":
      print isIFrame(header), frameNumber(header), timestamp(header), len(payload)
      header, payload = merger.extract()
    tmp = f.read( size )
    
