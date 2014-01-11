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
    if not self.buf.startswith("PaVE"):
      if "PaVE" in self.buf:
        self.buf = self.buf[ self.buf.index("PaVE") : ]


  def extract( self ):
    if len(self.buf) < 4+1+1+2+4:
      # at least struct of version and header and payload sizes must be ready
      return ""
    
    if not self.buf.startswith( "PaVE" ):
      return ""

    version, codec, headerSize, payloadSize = struct.unpack_from("BBHI", self.buf, 4 )
    if len(self.buf) < headerSize + payloadSize:
      return ""

    ret = self.buf[:headerSize + payloadSize]
    self.buf = self.buf[headerSize + payloadSize : ]
    return ret

if __name__ == "__main__":
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(2)

