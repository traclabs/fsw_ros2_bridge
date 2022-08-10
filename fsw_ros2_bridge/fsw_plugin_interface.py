from abc import ABC, abstractmethod

from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo


class FSWPluginInterface:
    def __init__(self, node):
        self._telem_info = []
        self._command_info = []
        self._node = node

    @abstractmethod
    def get_telemetry_message_info(self):
        pass

    @abstractmethod
    def get_command_message_info(self):
        pass

    @abstractmethod
    def get_latest_data(self, key):
        pass

    @abstractmethod
    def create_ros_msgs(self, msg_dir):
        pass

    @abstractmethod
    def get_msg_package(self):
        pass
