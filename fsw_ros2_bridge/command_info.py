

class CommandInfo():

    def __init__(self, key, msg_type, topic_name, callback_func, port):
        self._key = key
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._callback_func = callback_func
        self._port = port

    def get_key(self):
        return self._key

    def get_msg_type(self):
        return self._msg_type

    def get_topic_name(self):
        return self._topic_name

    def get_callback_func(self):
        return self._callback_func

    def set_callback_func(self, func):
        self._callback_func = func

    def get_port(self):
        return self._port
