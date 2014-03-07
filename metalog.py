#!/usr/bin/python
"""
  MetaLog - binding of multiple log files
"""

class MetaLog:
  def __init__( self, filename ):
    self.filename = filename
    self.f = open( self.filename )
  
  def getLog( self, prefix ):
    for line in self.f:
      print "LINE", line.strip()
      if line.startswith( prefix ):
        return line.split()[1].strip()
    return None # not found
