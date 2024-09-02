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
from tqdm import tqdm

from message import Message
from camera import Camera
from camera_configs import front, rear, left, right

'''
2 -> rear
3 -> front
4 -> left
5 -> right
'''

def image_save(bridge, std_time, img_list, camera, count, undistort = False, logging = False):

    time_offset = []

    for m in img_list:
        time_offset.append(abs(std_time - m.get_time()))

    min_index = time_offset.index(min(time_offset))
    msg = img_list[min_index].get_msg()

    img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    if undistort:
        h, w = img.shape[:2]
        new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera.get_camera_matrix(),
                                                            camera.get_distortion_coefficients(),
                                                            (w,h),
                                                            1,
                                                            (w,h))

        undistorted_image = cv2.undistort(img,
                                        camera.get_camera_matrix(),
                                        camera.get_distortion_coefficients(),
                                        None,
                                        new_camera_matrix)

        cv2.imwrite(os.path.join(camera.get_save_dir(), f'{count:06d}.png'), undistorted_image)
    else:
        cv2.imwrite(os.path.join(camera.get_save_dir(), f'{count:06d}.png'), img)

    with open(camera.get_image_names_dir(), 'a') as file:
        file.write(f'{camera.get_position()}/{count:06d}.png\n')

    if logging:
        with open(camera.get_geotagged_image_dir(), 'a') as file:
            file.write(f'{camera.get_position()}/{count:06d}.png\n')

def filtering(filtering_time, img_list):
    if img_list[0].get_time() < filtering_time:
        img_list.pop(0)
        filtering(filtering_time, img_list)

def main():

    BAG_FILE_PATH = '/media/moon/T7/2024-08-28-20-41-34_query.bag'
    POSE_TOPIC = '/lidar_points'

    front_camera = Camera(**front)
    rear_camera = Camera(**rear)
    left_camera = Camera(**left)
    right_camera = Camera(**right)

    # LOG = 'log.txt'
    # with open(LOG, 'w') as file:
    #     file.write(f'pose_time tx ty tz x y z w img1_time img2_time img3_time img4_time\n')

    topics = [POSE_TOPIC,
              front_camera.get_topic(),
              rear_camera.get_topic(),
              left_camera.get_topic(),
              right_camera.get_topic()]

    bag = rosbag.Bag(BAG_FILE_PATH, 'r')
    bridge = CvBridge()
    count = 0

    pose_list = []
    front_img_list = []
    rear_img_list = []
    left_img_list = []
    right_img_list = []

    message_list = []

    for topic, msg, time in tqdm(bag.read_messages(topics=topics)):

        message_list.append(Message(topic, msg, time))

        if topic == POSE_TOPIC:
            pose_list.append(Message(topic, msg, time))
        elif topic == front_camera.get_topic():
            front_img_list.append(Message(topic, msg, time))
        elif topic == rear_camera.get_topic():
            rear_img_list.append(Message(topic, msg, time))
        elif topic == left_camera.get_topic():
            left_img_list.append(Message(topic, msg, time))
        elif topic == right_camera.get_topic():
            right_img_list.append(Message(topic, msg, time))
        else:
            print("topic name is not matched.")

        # When all topics have been receieved.
        if len(pose_list) >= 2:
            std_time = pose_list[-1].get_time()

            bridge = CvBridge()

            image_save(bridge, std_time, front_img_list, front_camera, count)
            image_save(bridge, std_time, rear_img_list, rear_camera, count)
            image_save(bridge, std_time, left_img_list, left_camera, count)
            image_save(bridge, std_time, right_img_list, right_camera, count)

            # initialize
            filtering_time = pose_list.pop(0).get_time()
            filtering(filtering_time, front_img_list)
            filtering(filtering_time, rear_img_list)
            filtering(filtering_time, left_img_list)
            filtering(filtering_time, right_img_list)

            count += 1
        # cv2.imwrite(os.path.join(OUTPUT_DIR, "frame%06i.png" % count), cv_img)
        # print ("Wrote image %i" % count)

        # 1724827817 629 848 429
        # 1724827817 662 143 810
        # 1724827817 667 910 809
        # 1724827817 690 015 336

        # print(f'topic: {topic}')
        # print(f'msg: {type(msg)}')
        # print(f'time: {time}')


    bag.close()

if __name__ == '__main__':
    main()