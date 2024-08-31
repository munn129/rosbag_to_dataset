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
from camera import Camera
from camera_configs import front, rear, left, right

'''
2 -> rear
3 -> front
4 -> left
5 -> right
'''

def main():

    front_camera = Camera(front)
    rear_camera = Camera(rear)
    left_camera = Camera(left)
    right_camera = Camera(right)

    BAG_FILE_PATH = 'debug2.bag'
    FRONT_CAMERA = '/gmsl_camera/dev/video3/compressed'
    REAR_CAMERA = '/gmsl_camera/dev/video2/compressed'
    LEFT_CAMERA = '/gmsl_camera/dev/video4/compressed'
    RIGHT_CAMERA = '/gmsl_camera/dev/video5/compressed'
    POSE_TOPIC = '/kiss/odometry'

    IMAGE1_NAMES = 'img1.txt'
    IMAGE2_NAMES = 'img2.txt'

    GEOTAGGED_IMAGE1 = 'img11.txt'
    GEOTAGGED_IMAGE2 = 'img22.txt'

    LOG = 'log.txt'
    with open(LOG, 'r') as file:
        file.write(f'pose_time tx ty tz x y z w img1_time img1_dir img2_time img2_dir\n')

    OUTPUT_DIR = './output'
    if not os.path.exists(OUTPUT_DIR): os.mkdir(OUTPUT_DIR)
    FRONT_IMAGE_DIR = os.path.join(OUTPUT_DIR, 'front')
    if not os.path.exists(FRONT_IMAGE_DIR): os.mkdir(FRONT_IMAGE_DIR)
    REAR_IMAGE_DIR = os.path.join(OUTPUT_DIR, 'rear')
    if not os.path.exists(REAR_IMAGE_DIR): os.mkdir(REAR_IMAGE_DIR)
    LEFT_IMAGE_DIR = os.path.join(OUTPUT_DIR, 'left')
    if not os.path.exists(LEFT_IMAGE_DIR): os.mkdir(LEFT_IMAGE_DIR)
    RIGHT_IMAGE_DIR = os.path.join(OUTPUT_DIR, 'right')
    if not os.path.exists(RIGHT_IMAGE_DIR): os.mkdir(RIGHT_IMAGE_DIR)

    topics = [POSE_TOPIC, FRONT_CAMERA, REAR_CAMERA, LEFT_CAMERA, RIGHT_CAMERA]

    pose_flag = False
    front_image_flag = False
    rear_image_flag = False
    left_image_flag = False
    right_image_flag = False

    bag = rosbag.Bag(BAG_FILE_PATH, 'r')
    bridge = CvBridge()
    count = 0

    pose_list = []
    image1_list = []
    image2_list = []

    message_list = []

    for topic, msg, time in bag.read_messages(topics=topics):

        message_list.append(Message(topic, msg, time))

        if topic == POSE_TOPIC:
            pose_flag = True
            pose_list.append(Message(topic, msg, time))
        elif topic == FRONT_CAMERA:
            front_image_flag = True
            image1_list.append(Message(topic, msg, time))
        elif topic == REAR_CAMERA:
            rear_image_flag = True
            image2_list.append(Message(topic, msg, time))
        else:
            print("topic name is not matched.")

        # When all topics have been receieved.
        if pose_flag and front_image_flag:
            std_time = pose_list[-1].get_time()

            img1_time_list = []
            img2_time_list = []

            for m in image1_list:
                img1_time_list.append(abs(std_time - m.get_time()))
            
            for m in image2_list:
                img2_time_list.append(abs(std_time - m.get_time()))

            # time_approximate
            img1_min_index = img1_time_list.index(min(img1_time_list))
            img2_min_index = img2_time_list.index(min(img2_time_list))

            img1_msg = image1_list[img1_min_index].get_msg()
            img2_msg = image2_list[img2_min_index].get_msg()

            # save image
            cv_img1 = bridge.compressed_imgmsg_to_cv2(img1_msg, desired_encoding='passthrough')
            cv_img2 = bridge.compressed_imgmsg_to_cv2(img2_msg, desired_encoding='passthrough')
            cv2.imwrite(os.path.join(FRONT_IMAGE_DIR, f'{count:06d}.png'), cv_img1)
            cv2.imwrite(os.path.join(REAR_IMAGE_DIR, f'{count:06d}.png'), cv_img2)

            # initialize
            pose_flag = False
            front_image_flag = False
            rear_image_flag = False
            pose_list[:] = []
            image1_list[:] = []
            image2_list[:] = []

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

if __name__ == '__main__':
    main()