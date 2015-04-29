#!/usr/bin/python
"""
  Parse on-board collected binary data from /dev/ttyO1
  usage:
       ./parse_navdata.py <log file>
"""
# ref C-code navdata_measure_t is available at
# https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/boards/ardrone/navdata.h

import sys
import struct

NAVDATA_PACKET_SIZE = 60
NAVDATA_START_BYTE = 0x3A


def checksum( packet ):
    # does not match yet :(
    s = 0
    while len(packet) > 0:
        s += struct.unpack("<H", packet[:2])[0]
        print hex(s)
        packet = packet[2:]
    return s & 0xFFFF


def parseNavdata( data ):
    while len(data) > NAVDATA_PACKET_SIZE:
        packet = data[:NAVDATA_PACKET_SIZE]
        assert packet[0] == chr(NAVDATA_START_BYTE)
#        print checksum( packet[2:] )
        header, frameNumber = struct.unpack("HH", packet[:4])
        assert header == NAVDATA_START_BYTE
        ax,ay,az = struct.unpack("HHH", packet[4:10])
        vx,vy,vz = struct.unpack("hhh", packet[10:16])
        tempAcc, tempGyro = struct.unpack("HH", packet[16:20])
        sonar = struct.unpack("HHHHH", packet[20:30])
        time, value, ref = struct.unpack("HHH", packet[30:36])
        numEcho, sumEcho, gradient = struct.unpack("<HIh", packet[36:44])
        echoIni, pressure, tempPressure = struct.unpack("<HiH", packet[44:52])
        mx,my,mz = struct.unpack("hhh", packet[52:58])
        data = data[NAVDATA_PACKET_SIZE:]


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    parseNavdata( open(sys.argv[1],"rb").read() )

# vim: expandtab sw=4 ts=4 

