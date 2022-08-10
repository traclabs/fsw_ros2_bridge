

class TelemInfo():

    def __init__(self, key, msg_type, topic_name):
        self._key = key
        self._msg_type = msg_type
        self._topic_name = topic_name

    def get_key(self):
        return self._key

    def get_msg_type(self):
        return self._msg_type

    def get_topic_name(self):
        return self._topic_name
