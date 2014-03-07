#!/usr/bin/python
"""
  MetaLog - binding of multiple log files
"""
import os.path

class MetaLog:
  def __init__( self, filename ):
    self.filename = filename
    self.f = open( self.filename )
  
  def getLog( self, prefix ):
    for line in self.f:
      print "LINE", line.strip()
      if line.startswith( prefix ):
        ret = line.split()[1].strip()
        assert ret.startswith("logs/")
        return os.path.dirname( self.filename ) + os.sep + ret[4:]
    return None # not found
