import importlib
from fsw_ros2_bridge.fsw_plugin_interface import *

class FSWWrapper:
    def __init__(self, node, plugin=None):
        if plugin != None:
            self._plugin = importlib.import_module(plugin, ".").FSWPlugin(node)
        else:
            self._plugin = importlib.import_module("fsw_ros2_bridge.test_plugin", ".").FSWPlugin(node)

    def get_plugin(self):
        return self._plugin

    def get_msg_package(self):
        return self._plugin.get_msg_package()
