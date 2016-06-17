#!/usr/bin/python

import lcm
import drc
from bot_core.utime_t import utime_t
import sys
import time

msg = utime_t()
msg.utime = time.time()*1000*1000
lc = lcm.LCM()
lc.publish(sys.argv[1], msg.encode())