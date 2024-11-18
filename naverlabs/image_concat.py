import os
import cv2
import numpy as np

from pathlib import Path

dataset_dir = Path('/media/moon/T7 Shield/naverlabs_undistort/2019-12-03_10-54-54')
query_dir = Path('/media/moon/T7 Shield/naverlabs_undistort/2019-12-03_11-43-39')

sensors = sorted(list(query_dir.glob('*')))

# dataset_image = sorted(list(dataset_dir.glob('**/*.jpg')))
# query_image = sorted(list(query_dir.glob('**/*.jpg')))



def main():
    imagenames_all = []

    for sensor in sensors:
        postfix = str(sensor).split('/')[-1]
        imagenames_all.append(sorted(list(query_dir.glob(f'{postfix}/*.jpg'))))

    postfix = str(sensors[0]).split('/')[-1]
    image_len = len(list(query_dir.glob(f'{postfix}/*.jpg')))

    for i in range(image_len):
        img_1 = cv2.imread(str(imagenames_all[0][i]))
        img_2 = cv2.imread(str(imagenames_all[1][i]))
        img_3 = cv2.imread(str(imagenames_all[2][i]))
        img_4 = cv2.imread(str(imagenames_all[3][i]))
        img_5 = cv2.imread(str(imagenames_all[4][i]))
        img_6 = cv2.imread(str(imagenames_all[5][i]))
        top = np.hstack((img_1, img_2, img_3))
        bottom = np.hstack((img_4, img_5, img_6))
        img = np.vstack((top, bottom))

        img_name = str(imagenames_all[0][i]).split('/')[-1]
        cv2.imwrite(f'{str(query_dir)}/concat/{img_name}', img)


if __name__ == '__main__':
    main()