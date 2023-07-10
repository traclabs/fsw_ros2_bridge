"""
.. module:: fsw_ros2_bridge.fsw_plugin_interface
   :synopsis: Defines the interface each plugin must implement

.. moduleauthor:: Stephen Hart

"""

from abc import abstractmethod


class FSWPluginInterface:
    """This class defines the interface for each plugin.

    Attributes
    ----------
    telem_info : list
        list of TelemInfo objects
    command_info : list
        list of CommandInfo objects
    node : rosnode
        the ROS2 node

    Methods
    -------
    get_telemetry_message_info():
        Returns a list of telemetry message types.
    get_command_message_info():
        Returns a list of command message types.
    get_latest_data(key):
        Returns the value of the requested telemetry object.
    create_ros_msgs(msg_dir):
        Returns a list of ROS2 messages.
    get_msg_package():
        Returns the ROS2 package containing the messages.
    """

    def __init__(self, node):
        '''
        Initializes the attributes for the plugin object.

            Parameters:
                    node (rosnode): The ROS2 node
        '''
        self._telem_info = []
        self._command_info = []
        self._node = node

    @abstractmethod
    def get_telemetry_message_info(self):
        '''
        Returns a list of telemetry message types.

            Returns:
                    telem_info (list): List of TelemInfo objects
        '''
        pass

    @abstractmethod
    def get_command_message_info(self):
        '''
        Returns a list of command message types.

            Returns:
                    command_info (list): List of CommandInfo objects
        '''
        pass

    @abstractmethod
    def get_buffered_data(self, key, clear=True):
        '''
        Returns the value of the requested telemetry object.

            Parameters:
                    key (str): The key that identifies the telemetry requested
                    clear (bool): If the data queue should be cleared

            Returns:
                    value (dict): Dictionary containing the current telemetry value
        '''
        pass

    @abstractmethod
    def create_ros_msgs(self, msg_dir):
        '''
        Returns a list of ROS2 messages

            Parameters:
                    msg_dir (str): The directory containing the ROS2 messages

            Returns:
                    msg_list (list): List of ROS2 message objects
        '''
        pass

    @abstractmethod
    def get_msg_package(self):
        '''
        Returns the ROS2 package containing the messages

            Returns:
                    msg_pkg (str): The name of the ROS2 package
        '''
        pass
