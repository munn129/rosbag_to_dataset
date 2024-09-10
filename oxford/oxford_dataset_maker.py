import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from sdk.camera_model import CameraModel
from sdk.image import load_image

import cv2
import numpy as np

def image_undistort(image_dir):
    model = './camera-models'
    
    camera_model = CameraModel(model, image_dir)

    return load_image(image_dir, camera_model)

def is_synced(cam, ins, std_time_gap, save_list) -> bool:
    if (len(save_list) > 1) and save_list[-1] == cam:
        return False

    if abs(int(cam) - int(ins)) < std_time_gap:
        save_list.append(cam)
        return True
    else:
        return False
    
def image_concat(images: list):
    '''
    image concat

    sequence of images: list -> front, rear, left, right
    front | rear  => top
    -------------
    left  | right => bottom

    front: 960*1280
    left, right, rear: 1024*1024
    '''

    # front image resize to size of rear image
    images[0] = cv2.resize(images[0], images[1].shape[:2])

    top = np.hstack((images[0], images[1]))
    bottom = np.hstack((images[2], images[3]))

    return np.vstack((top, bottom))

def main():
    # params
    # setting input directory
    date = '0514'
    detail_date = '2014-05-14-13-46-12'
    dataset_root_dir = '/media/moon/moon_ssd/moon ubuntu'

    # timestamp,ins_status,latitude,longitude,altitude,northing,easting,down,utm_zone,velocity_north,velocity_east,velocity_down,roll,pitch,yaw
    ins_csv = f'{dataset_root_dir}/oxford/{date}/{detail_date}_gps/{detail_date}/gps/ins.csv'
    front_cam_prefix = f'{dataset_root_dir}/oxford/{date}/{detail_date}_stereo_centre_01/{detail_date}'
    left_cam_prefix = f'{dataset_root_dir}/oxford/{date}/{detail_date}_mono_left_01/{detail_date}'
    right_cam_prefix = f'{dataset_root_dir}/oxford/{date}/{detail_date}_mono_right_01/{detail_date}'
    rear_cam_prefix = f'{dataset_root_dir}/oxford/{date}/{detail_date}_mono_rear_01/{detail_date}'

    front_cam_timestamp = os.path.join(front_cam_prefix, 'stereo.timestamps')
    left_cam_timestamp = os.path.join(left_cam_prefix, 'mono_left.timestamps')
    right_cam_timestamp = os.path.join(right_cam_prefix, 'mono_right.timestamps')
    rear_cam_timestamp = os.path.join(rear_cam_prefix, 'mono_rear.timestamps')

    # setting output directory
    output_prefix = os.path.join(dataset_root_dir, 'post_oxford')

    front_post_fix = os.path.join(date, 'front')
    left_post_fix = os.path.join(date, 'left')
    right_post_fix = os.path.join(date, 'right')
    rear_post_fix = os.path.join(date, 'rear')
    concat_post_fix = os.path.join(date, 'concat')

    front_output = os.path.join(output_prefix, front_post_fix)
    left_output = os.path.join(output_prefix, left_post_fix)
    right_output = os.path.join(output_prefix, right_post_fix)
    rear_output = os.path.join(output_prefix, rear_post_fix)
    concat_output = os.path.join(output_prefix, concat_post_fix)

    cam_timestamp_dir = [front_cam_timestamp, left_cam_timestamp, right_cam_timestamp, rear_cam_timestamp]

    # read ins.csv and save timestamp and gt(lat, lon, yaw)
    ins_timestamp = []
    ins_gt = [] # (latitude, longitude, yaw)
    with open(ins_csv, 'r') as file:
        for line in file:
            if line[0] == 't': continue
            content = line.split(',')
            if content[1] == 'INS_SOLUTION_GOOD':
                ins_timestamp.append(content[0])
                ins_gt.append((content[2], content[3],content[-1]))
    
    print('------------------------PROGRESS REPORT----------------------------')
    print(f'length of ins list: {len(ins_timestamp)}')
    print(f'length of gt list: {len(ins_gt)}')

    # read camera timestamps and save with list
    front_time_list = []
    left_time_list = []
    right_time_list = []
    rear_time_list = []

    cam_time_list = [front_time_list, left_time_list, right_time_list, rear_time_list]

    for input, output in zip(cam_timestamp_dir, cam_time_list):
        with open(input, 'r') as file:
            for line in file:
                output.append(line.split(' ')[0])

    print('------------------------PROGRESS REPORT----------------------------')
    print(f'length of front time list: {len(front_time_list)}')
    print(f'length of left time list: {len(left_time_list)}')
    print(f'length of right time list: {len(right_time_list)}')
    print(f'length of rear time list: {len(rear_time_list)}')

    # time synced list
    ins_save_list = []
    gt = []
    front_save_list = []
    left_save_list = []
    right_save_list = []
    rear_save_list = []

    front_flag = False
    left_flag = False
    right_flag = False
    rear_flag = False

    std_time_gap = 30000 # 30ms

    for idx, ins in enumerate(ins_timestamp):

        for cam in front_time_list:
            if is_synced(cam, ins, std_time_gap, front_save_list):
                front_flag = True
                break

        for cam in left_time_list:
            if is_synced(cam, ins, std_time_gap, left_save_list):
                left_flag = True
                break
        
        for cam in right_time_list:
            if is_synced(cam, ins, std_time_gap, right_save_list):
                right_flag = True
                break

        for cam in rear_time_list:
            if is_synced(cam, ins, std_time_gap, rear_save_list):
                rear_flag = True
                break

        # sync check
        # if all flags are True, it is synced.
        if front_flag and left_flag and right_flag and rear_flag:
            ins_save_list.append(ins)
            gt.append(ins_gt[idx])

        # if each images(multi direction) is not synced, pop()
        else:
            if front_flag:
                front_save_list.pop()
            
            if left_flag:
                left_save_list.pop()

            if right_flag:
                right_save_list.pop()

            if rear_flag:
                rear_save_list.pop()
    
        # flag initialize
        front_flag = False
        left_flag = False
        right_flag = False
        rear_flag = False

    print('------------------------PROGRESS REPORT----------------------------')
    print(f'length of ins save list: {len(ins_save_list)}')
    print(f'length of gt list: {len(gt)}')
    print(f'length of front save list: {len(front_save_list)}')
    print(f'length of left save list: {len(left_save_list)}')
    print(f'length of right save list: {len(right_save_list)}')
    print(f'length of rear save list: {len(rear_save_list)}')

    front_image_dir = os.path.join(front_cam_prefix, 'stereo', 'centre')
    left_image_dir = os.path.join(left_cam_prefix, 'mono_left')
    right_image_dir = os.path.join(right_cam_prefix, 'mono_right')
    rear_image_dir = os.path.join(rear_cam_prefix, 'mono_rear')

    is_imshow = False

    # save for dataset
    for idx in range(len(ins_save_list)):

        # image undistort
        front_image = image_undistort(os.path.join(front_image_dir, str(front_save_list[idx]) + '.png'))
        left_image = image_undistort(os.path.join(left_image_dir, str(left_save_list[idx]) + '.png'))
        right_image = image_undistort(os.path.join(right_image_dir, str(right_save_list[idx]) + '.png'))
        rear_image = image_undistort(os.path.join(rear_image_dir, str(rear_save_list[idx]) + '.png'))

        # image concat
        concat_image = image_concat([front_image, rear_image, left_image, right_image])

        if is_imshow:
            while True:
                cv2.namedWindow('front', cv2.WINDOW_NORMAL)
                cv2.imshow('front', front_image)

                if cv2.waitKey(1) & 0xFF == ord('q'): break

            while True:
                cv2.namedWindow('left', cv2.WINDOW_NORMAL)
                cv2.imshow('left', left_image)

                if cv2.waitKey(1) & 0xFF == ord('q'): break
            
            while True:
                cv2.namedWindow('right', cv2.WINDOW_NORMAL)
                cv2.imshow('right', right_image)

                if cv2.waitKey(1) & 0xFF == ord('q'): break

            while True:
                cv2.namedWindow('rear', cv2.WINDOW_NORMAL)
                cv2.imshow('rear', rear_image)

                if cv2.waitKey(1) & 0xFF == ord('q'): break        

            while True:
                cv2.namedWindow('concat', cv2.WINDOW_NORMAL)
                cv2.imshow('concat', concat_image)

                if cv2.waitKey(1) & 0xFF == ord('q'): break

            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()