import numpy as np

front = {
    'topic' : '/gmsl_camera/dev/video3/compressed',
    'position' : 'front',
    'camera_matrix' : np.array([]),
    'distortion_coefficients' : np.array([])
}

rear = {
    'topic' : '/gmsl_camera/dev/video2/compressed',
    'position' : 'rear',
    'camera_matrix' : np.array([]),
    'distortion_coefficients' : np.array([])
}

left = {
    'topic' : '/gmsl_camera/dev/video4/compressed',
    'position' : 'left',
    'camera_matrix' : np.array([]),
    'distortion_coefficients' : np.array([])
}

right = {
    'topic' : '/gmsl_camera/dev/video5/compressed',
    'position' : 'right',
    'camera_matrix' : np.array([]),
    'distortion_coefficients' : np.array([])
}