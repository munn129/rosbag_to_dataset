import os

class Camera:
    def __init__(self, **kwargs) -> None:
        self.topic = kwargs['topic']
        self.position = kwargs['position']
        self.camera_metrix = kwargs['camera_matrix']
        self.distortion_coefficients = kwargs['distortion_coefficients']

        self.output_dir = os.path.join('.', self.position)
        if not os.path.exists(self.output_dir): os.mkdir(self.output_dir)
        self.image_names = os.path.join(self.output_dir, f'{self.position}_names.txt')
        with open(self.image_names, 'w') as file:
            file.write(f'')
        
        self.geotagged_image = os.path.join(self.output_dir, f'{self.position}.txt')
        with open(self.geotagged_image, 'w') as file:
            file.write(f'')

    def get_topic(self) -> str:
        return self.topic
    
    def get_position(self) -> str:
        return self.position
    
    def get_camera_metrix(self):
        return self.camera_metrix
    
    def get_distortion_coefficients(self):
        return self.distortion_coefficients
    
    def get_save_dir(self):
        return self.output_dir
    
    def get_image_names_dir(self):
        return self.image_names
    
    def get_geotagged_image_dir(self):
        return self.geotagged_image