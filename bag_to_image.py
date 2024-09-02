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

def image_save(bridge, std_time, img_list, camera, count):

    time_offset = []

    for m in img_list:
        time_offset.append(abs(std_time - m.get_time()))

    min_index = time_offset.index(min(time_offset))
    msg = img_list[min_index].get_msg()

    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imwrite(os.path.join(camera.get_save_dir(), f'{count:06d}.png'), img)

def filtering(filtering_time, img_list):
    if img_list[0].get_time() < filtering_time:
        img_list.pop(0)
        filtering(filtering_time, img_list)

def main():

    BAG_FILE_PATH = '/media/moon/T7/2024-08-28-20-17-09_dataset.bag'
    POSE_TOPIC = '/lidar_points'

    front_camera = Camera(**front)
    rear_camera = Camera(**rear)
    left_camera = Camera(**left)
    right_camera = Camera(**right)

    LOG = 'log.txt'
    with open(LOG, 'w') as file:
        file.write(f'pose_time tx ty tz x y z w img1_time img1_dir img2_time img2_dir\n')

    topics = [POSE_TOPIC,
              front_camera.get_topic(),
              rear_camera.get_topic(),
              left_camera.get_topic(),
              right_camera.get_topic()]

    pose_flag = False
    front_image_flag = False
    rear_image_flag = False
    left_image_flag = False
    right_image_flag = False

    bag = rosbag.Bag(BAG_FILE_PATH, 'r')
    bridge = CvBridge()
    count = 0

    pose_list = []
    front_img_list = []
    rear_img_list = []
    left_img_list = []
    right_img_list = []

    message_list = []

    for topic, msg, time in bag.read_messages(topics=topics):

        message_list.append(Message(topic, msg, time))

        if topic == POSE_TOPIC:
            pose_flag = True
            pose_list.append(Message(topic, msg, time))
        elif topic == front_camera.get_topic():
            front_image_flag = True
            front_img_list.append(Message(topic, msg, time))
        elif topic == rear_camera.get_topic():
            rear_image_flag = True
            rear_img_list.append(Message(topic, msg, time))
        elif topic == left_camera.get_topic():
            left_image_flag = True
            left_img_list.append(Message(topic, msg, time))
        elif topic == right_camera.get_topic():
            right_image_flag = True
            right_img_list.append(Message(topic, msg, time))
        else:
            print("topic name is not matched.")

        # When all topics have been receieved.
        # if pose_flag and front_image_flag and rear_image_flag and left_image_flag and right_image_flag:
        if len(pose_list) >= 2 and count >= 1:
            std_time = pose_list[-1].get_time()

            bridge = CvBridge()

            image_save(bridge, std_time, front_img_list, front, count)
            image_save(bridge, std_time, rear_img_list, rear, count)
            image_save(bridge, std_time, left_img_list, left, count)
            image_save(bridge, std_time, right_img_list, right, count)

            # initialize
            filtering_time = pose_list.pop(0).get_time()
            filtering(filtering_time, front_img_list)
            filtering(filtering_time, rear_img_list)
            filtering(filtering_time, left_img_list)
            filtering(filtering_time, right_img_list)

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