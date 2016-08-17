#!/usr/bin/python

import lcm
import bot_core
from bot_core.raw_t import raw_t

msg = raw_t()
msg.utime = 0
msg.data = "filename.txt"
msg.length = len(msg.data)
lc = lcm.LCM()
lc.publish("IRB140_ESTIMATOR_SAVE_POINTCLOUD", msg.encode())