import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from sdk.camera_model import CameraModel
from sdk.image import load_image

import cv2

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
    left_cam_timestamp = os.path.join(left_cam_prefix, 'stereo.timestamps')
    right_cam_timestamp = os.path.join(right_cam_prefix, 'stereo.timestamps')
    rear_cam_timestamp = os.path.join(rear_cam_prefix, 'stereo.timestamps')

    # setting output directory
    output_prefix = os.path.join(dataset_root_dir, 'post_oxford')

    front_output = os.path.join(output_prefix, f'{date}/front')
    left_output = os.path.join(output_prefix, f'{date}/left')
    right_output = os.path.join(output_prefix, f'{date}/right')
    rear_output = os.path.join(output_prefix, f'{date}/rear')
    concat_output = os.path.join(output_prefix, f'{date}/concat')

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

if __name__ == '__main__':
    main()