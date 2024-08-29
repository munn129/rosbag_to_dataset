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
    BAG_FILE_PATH = 'debug2.bag'
    IMAGE_1_TOPIC = '/clpe_ros/cam_0/image_raw'
    IMAGE_2_TOPIC = '/clpe_ros/cam_2/image_raw'
    POSE_TOPIC = '/kiss/odometry'
    OUTPUT_DIR = './output'

    topics = [POSE_TOPIC, IMAGE_1_TOPIC, IMAGE_2_TOPIC]

    bag = rosbag.Bag(BAG_FILE_PATH, "r")
    bridge = CvBridge()
    count = 0
    for topic, msg, time in bag.read_messages(topics=topics):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        

        cv2.imwrite(os.path.join(OUTPUT_DIR, "frame%06i.png" % count), cv_img)
        print ("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()