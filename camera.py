import os

class Camera:
    def __init__(self, **kwargs) -> None:
        self.topic = kwargs['topic']
        self.position = kwargs['position']
        self.param = kwargs['param']

        self.output_dir = os.path.join('.', self.position)
        if not os.path.exists(self.output_dir): os.mkdir(self.output_dir)
        self.image_names = os.path.join(self.output_dir, f'{self.position}_names.txt')
        self.geotagged_image = os.path.join(self.output_dir, f'{self.position}.txt')

    def get_topic(self) -> str:
        return self.topic
    
    def get_position(self) -> str:
        return self.position
    
    def get_param(self):
        return self.param