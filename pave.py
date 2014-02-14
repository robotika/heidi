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
    packet = merger.extract()
    while packet != "":
      print len(packet)
      packet = merger.extract()
    tmp = f.read( size )
    
