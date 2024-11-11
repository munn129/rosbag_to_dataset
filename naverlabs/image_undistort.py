import os
import cv2
import numpy as np

from pathlib import Path

from camera_params import camera_params

dataset_dir = Path('/media/moon/T7 Shield/naverlabs_datasets/GangnamStation_B2/GangnamStation_B2_release_mapping/GangnamStation/B2/release/mapping/sensors/records_data/2019-12-03_10-54-54')
query_dir = Path('/media/moon/T7 Shield/naverlabs_datasets/GangnamStation_B2/GangnamStation_B2_release_mapping/GangnamStation/B2/release/mapping/sensors/records_data/2019-12-03_11-43-39')

dataset_imagenames_dir = sorted(list(dataset_dir.glob('**/*.jpg')))
query_imagenames_dir = sorted(list(query_dir.glob('**/*.jpg')))

dataset_save_dir = '/media/moon/T7 Shield/naverlabs_undistort/2019-12-03_10-54-54'
query_save_dir = '/media/moon/T7 Shield/naverlabs_undistort/2019-12-03_11-43-39'

dataset_imagename_only = [str(i).split('/')[-1] for i in dataset_imagenames_dir]
query_imagename_only = [str(i).split('/')[-1] for i in query_imagenames_dir]

def main():
    
    for i in range(len(dataset_imagename_only)):
        image_path = dataset_imagenames_dir[i] # Path
        imagename = dataset_imagename_only[i] # str
        id = imagename.split('_')[0]
        camera_param = camera_params[id]

        camera_matrix = np.array([
            [camera_param['fx'], 0, camera_param['cx']],
            [0, camera_param['fy'], camera_param['cy']],
            [0,0,1]
        ])
        
        distort_coefficient = np.array([camera_param['k1'], camera_param['k2'], camera_param['p1'], camera_param['p2']])

        distorted_image = cv2.imread(str(image_path))

        undistorted_image = cv2.undistort(distorted_image, camera_matrix, distort_coefficient)

        save_dir = os.path.join(dataset_save_dir, id)

        if not(os.path.isdir(save_dir)):
            os.mkdir(save_dir)

        cv2.imwrite(os.path.join(save_dir, imagename.split('_')[-1]), undistorted_image)

if __name__ == '__main__':
    main()