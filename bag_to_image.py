#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Extract images from a rosbag.
"""

import os

import cv2

import rosbag
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from message import Message

def main():
    BAG_FILE_PATH = 'debug2.bag'
    IMAGE_1_TOPIC = '/clpe_ros/cam_0/image_raw'
    IMAGE_2_TOPIC = '/clpe_ros/cam_2/image_raw'
    POSE_TOPIC = '/kiss/odometry'
    OUTPUT_DIR = './output'

    COMPRESSED = False

    if not os.path.exists(OUTPUT_DIR): os.mkdir(OUTPUT_DIR)
    IMAGE_1_DIR = os.path.join(OUTPUT_DIR, 'front')
    if not os.path.exists(IMAGE_1_DIR): os.mkdir(IMAGE_1_DIR)
    IMAGE_2_DIR = os.path.join(OUTPUT_DIR, 'left')
    if not os.path.exists(IMAGE_2_DIR): os.mkdir(IMAGE_2_DIR)

    topics = [POSE_TOPIC, IMAGE_1_TOPIC, IMAGE_2_TOPIC]

    pose__flag = False
    image1_flag = False
    image2_flag = False

    bag = rosbag.Bag(BAG_FILE_PATH, 'r')
    bridge = CvBridge()
    count = 0

    pose_topic_list = []
    image1_list = []
    image2_list = []

    last_pose = ''

    message_list = []

    for topic, msg, time in bag.read_messages(topics=topics):

        message_list.append(Message(topic, msg, time))

        if topic == POSE_TOPIC:
            pose_flag = True
        elif topic == IMAGE_1_TOPIC:
            image1_flag = True
        elif topic == IMAGE_2_TOPIC:
            image2_flag = True
        else:
            print("topic name is not matched.")

        if pose_flag and image1_flag and image2_flag:
            pass

        if COMPRESSED:
            cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        else:
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # cv2.imwrite(os.path.join(OUTPUT_DIR, "frame%06i.png" % count), cv_img)
        # print ("Wrote image %i" % count)

        # 1724827817 629 848 429
        # 1724827817 662 143 810
        # 1724827817 667 910 809
        # 1724827817 690 015 336

        print(f'topic: {topic}')
        print(f'msg: {type(msg)}')
        print(f'time: {time}')

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()