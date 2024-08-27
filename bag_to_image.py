#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Extract images from a rosbag.
"""

import os

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    BAG_FILE_PATH = 'debug.bag'
    IMAGE_TOPIC = '/gmsl_camera/dev/video0/compressed'
    OUTPUT_DIR = './output'

    bag = rosbag.Bag(BAG_FILE_PATH, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=IMAGE_TOPIC):
        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")

        cv2.imwrite(os.path.join(OUTPUT_DIR, "frame%06i.png" % count), cv_img)
        print ("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()