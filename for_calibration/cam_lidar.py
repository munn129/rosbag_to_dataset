#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
import rosbag
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

# def save_pcd(cloud, timestamp, path):
#     p = pcl.PointCloud(np.array(list(pc2.read_points(cloud)), dtype=np.float32)[:, 0:3])
#     p.to_file(path + '/pcd' + '_' + "{:.5f}".format(timestamp) + '.pcd')

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
    
    topics = [left_image, right_image, lidar]

    bag = rosbag.Bag(bag_file, 'r')

    cnt = 0

    for topic, msg, time in bag.read_messages(topics = topics):
        print(f'time: {time}, topic: {topic}')
        if topic == lidar:
            save_pcd(msg, f'{cnt:06d}.pcd')
            cnt += 1

if __name__ == '__main__':
    main()