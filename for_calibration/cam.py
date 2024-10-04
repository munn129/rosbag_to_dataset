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

    bridge = CvBridge()

    r_count = 0
    l_count = 0

    init_time = 0

    for topic, msg, time in bag.read_messages(topics = topics):

        time = int(str(time))

        if time - init_time > 1000000000:
        
            if topic == left_image:
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

                cv2.imwrite(f'{left_jpg}/{l_count:06d}.jpg', img)

                print(f'{left_jpg}/{l_count:06d}.jpg is saved')

                l_count += 1

            if topic == right_image:
                img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

                cv2.imwrite(f'{right_jpg}/{r_count:06d}.jpg', img)

                print(f'{right_jpg}/{r_count:06d}.jpg is saved')

                r_count += 1

            init_time = time

if __name__ == '__main__':
    main()