import importlib


class FSWWrapper:
    def __init__(self, node, plugin=None):
        if plugin is not None:
            self._plugin = importlib.import_module(plugin, ".").FSWPlugin(node)
        else:
            pn = "fsw_ros2_bridge.test_plugin"
            self._plugin = importlib.import_module(pn, ".").FSWPlugin(node)

    def get_plugin(self):
        return self._plugin

    def get_msg_package(self):
        return self._plugin.get_msg_package()
