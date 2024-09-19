import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from sdk.camera_model import CameraModel
from sdk.image import load_image

import cv2

'''
left, right, rear: 1024*1024
front: 960*1280
'''

def main():
    model_dir = './camera-models/'
    images_dir_prefix = '/media/moon/moon_ssd/moon_ubuntu/oxford'
    # images_dir = f'{images_dir_prefix}/0514/2014-05-14-13-46-12_stereo_centre_01/2014-05-14-13-46-12/stereo/centre/1400075267901002.png'
    images_dir = f'{images_dir_prefix}/0828/stereo_centre/1440751825385541.png'

    camera_model = CameraModel(model_dir, images_dir)

    img = load_image(images_dir, camera_model)
    print(img.shape)

    while True:

        cv2.imshow('tmp', img)

        if cv2.waitKey(1) & 0xFF == ord('q'): break

if __name__ == '__main__':
    main()