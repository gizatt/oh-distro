#!/usr/bin/python

import lcm
import drc
from bot_core import rigid_transform_t
import sys
import time

if len(sys.argv) != 9:
	print "Usage: publish_transform <channel> <x> <y> <z> <w> <x> <y> <z>"

channel = sys.argv[1]
x = float(sys.argv[2])
y = float(sys.argv[3])
z = float(sys.argv[4])
ww = float(sys.argv[5])
wx = float(sys.argv[6])
wy = float(sys.argv[7])
wz = float(sys.argv[8])

lc = lcm.LCM()
msg = rigid_transform_t();
msg.utime = 0;
msg.trans = [x, y, z];
msg.quat = [ww, wx, wy, wz];

while (1):
	lc.publish(channel, msg.encode())
	time.sleep(0.1)