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