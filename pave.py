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
    ret = self.buf
    self.buf = ""
    return ret

if __name__ == "__main__":
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(2)

