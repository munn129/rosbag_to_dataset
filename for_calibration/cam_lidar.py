#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
import rosbag
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

class Message:
    def __init__(self, topic_name, msg, time) -> None:
        self.topic_name = topic_name
        self. msg = msg
        self.time = time

    def get_topic_name(self) -> str:
        return str(self.topic_name)
    
    def get_msg(self):
        return self.msg
    
    def get_time(self) -> int:
        return int(str(self.time))

def save_pcd (pcd_msg, path):

    points_list = []

    for point in pc2.read_points(pcd_msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans = True):
        points_list.append([point[0], point[1], point[2]])

    cloud_np = np.array(points_list, dtype = np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np)

    o3d.io.write_point_cloud(path, pcd)

def main():
    bag_file = '/home/moon/Documents/for_calibration/2024-09-30-17-44-16.bag'

    left_image = '/stereo/left/image_color/compressed'
    right_image = '/stereo/right/image_color/compressed'
    lidar = '/lidar_points'

    left_image_save_dir = '/home/moon/Documents/for_calibration/left_images'
    right_image_save_dir = '/home/moon/Documents/for_calibration/right_images'
    left_jpg = '/home/moon/Documents/for_calibration/left_jpg'
    right_jpg = '/home/moon/Documents/for_calibration/right_jpg'
    pcd_save_dir = '/home/moon/Documents/for_calibration/lidar_pcd'
    
    topics = [left_image, right_image, lidar]

    bag = rosbag.Bag(bag_file, 'r')

    msg_dict = {
        'lidar' : [],
        'left_cam' : [],
        'right_cam' : []
    }

    cnt = 0

    tmp = []

    critia = 10000000

    for topic, msg, time in bag.read_messages(topics = topics):
        # print(f'time: {time}, topic: {topic}')
        
        if topic == lidar:
            msg_dict['lidar'].append(Message(topic, msg, time))
        elif topic == left_image:
            msg_dict['left_cam'].append(Message(topic, msg, time))
        elif topic == right_image:
            msg_dict['right_cam'].append(Message(topic, msg, time))
        else:
            print('topic name is not matched')
        
if __name__ == '__main__':
    main()