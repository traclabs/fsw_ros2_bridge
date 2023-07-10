"""
.. module:: fsw_ros2_bridge.fsw_wrapper
   :synopsis: Defines the interface each plugin must implement

.. moduleauthor:: Stephen Hart

"""

import importlib


class FSWWrapper:
    """
    This class is a wrapper class that holds a plugin.

    Attributes
    ----------
    plugin : FSWPluginInterface
        the plugin that is running

    Methods
    -------
    get_plugin():
        Returns the running plugin.
    get_msg_package():
        Returns the ROS2 message package for this plugin.
    """
    def __init__(self, node, plugin=None):
        """
        Initialize the wrapper with the given plugin.

            Parameters:
                    plugin (FSWPluginInterface): The plugin to run
        """
        if plugin is not None:
            self._plugin = importlib.import_module(plugin, ".").FSWPlugin(node)
        else:
            pn = "fsw_ros2_bridge.test_plugin"
            self._plugin = importlib.import_module(pn, ".").FSWPlugin(node)

    def get_plugin(self):
        """
        Get the running plugin.

            Returns:
                    plugin (FSWPluginInterface): The running plugin
        """
        return self._plugin

    def get_msg_package(self):
        """
        Get the ROS2 message package for the running plugin.

            Returns:
                    msg_pkg (str): The ROS2 message package for the running plugin
        """
        return self._plugin.get_msg_package()
