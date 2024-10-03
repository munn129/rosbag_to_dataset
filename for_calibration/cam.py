#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

def main():
    bag_file = '/home/moon/Documents/for_calibration/2024-09-30-17-44-16.bag'

    left_image = '/stereo/left/image_color/compressed'
    right_image = '/stereo/right/image_color/compressed'

    left_jpg = '/home/moon/Documents/for_calibration/left_jpg'
    right_jpg = '/home/moon/Documents/for_calibration/right_jpg'
    
    topics = [left_image, right_image]

    bag = rosbag.Bag(bag_file, 'r')

    left_list = []
    right_list = []


    for topic, msg, time in bag.read_messages(topics = topics):
        pass

if __name__ == '__main__':
    main()