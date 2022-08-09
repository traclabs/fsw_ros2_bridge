from fsw_ros2_bridge.fsw_plugin_interface import *


class FSWPlugin(FSWPluginInterface):
    def get_telemetry_message_info(self):
        pass

    def get_command_message_info(self):
        pass

    def get_latest_data(self, key):
        pass

    def get_msg_package(self):
        pass
