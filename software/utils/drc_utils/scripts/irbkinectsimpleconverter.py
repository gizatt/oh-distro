#!/usr/bin/env python

"""
Author: geronm@mit.edu
Converts image_t messages from KINECT_RGB channel into size-1 images_t messages on KINECT_RGB_IMAGES channel.
"""

import lcm
import drc
from bot_core import image_t, images_t

#not needed; cosmetic
import sys

def convertNSend(channel, data):
    global lc
    msgIn = image_t.decode(data)
    msgOut = images_t()

    # Put the 1 image into a size-1 images_t message
    msgOut.utime = msgIn.utime
    msgOut.n_images = 1
    msgOut.image_types = [msgOut.LEFT]
    msgOut.images = [msgIn]

    lc.publish("CAMERA", msgOut.encode())

    print '.',
    sys.stdout.flush()


if __name__ == "__main__":
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lc.subscribe("KINECT_RGB", convertNSend)
    print "Convertor Setup Success.  Running..."
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        print "KeyboardInterrupt detected.  Convertor Terminated"    
