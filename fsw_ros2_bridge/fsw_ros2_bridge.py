"""
.. module:: fsw_ros2_bridge.fsw_ros2_bridge
   :synopsis: The top level of the bridge

.. moduleauthor:: Stephen Hart

"""

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory

import os
import importlib
import json
import glob

import fsw_ros2_bridge.fsw_wrapper
from fsw_ros2_bridge_msgs.srv import GetMessageInfo
from fsw_ros2_bridge_msgs.srv import SetMessageInfo
from fsw_ros2_bridge_msgs.srv import GetPluginInfo
from fsw_ros2_bridge_msgs.msg import MessageInfo


class FSWBridge(Node):
    """
    This class is the top level class.

    Attributes
    ----------
    dict_file : str
        JSON file containing message dictionary definitions
    message_info : list
        List of type MessageInfo containing all telemetry and command messages
    msg_dict : dictionary
        Dictionary mapping for message_info

    Methods
    -------
    """

    def __init__(self):
        '''
        Initializes attributes and ROS2 node information.
        '''
        super().__init__('fsw_ros2_bridge')

        self._dict_file = ""
        self._message_info = []
        self._msg_dict = {}

        # default plugin param, will be set/loaded from app config file
        self.declare_parameter('plugin_name', 'fsw_ros2_bridge.test_plugin')
        self._plugin_name = self.get_parameter('plugin_name').get_parameter_value().string_value
        self._plugin_pkg_name = self._plugin_name.split('.')[0]

        self.get_logger().warn("================================================================")
        self.get_logger().warn("Using FSW Plugin: " + self._plugin_name)
        self.get_logger().warn("================================================================")
        self._fsw = fsw_ros2_bridge.fsw_wrapper.FSWWrapper(self, self._plugin_name)
        self._msg_pkg = self._fsw.get_msg_package()

        self.get_message_info_srv = self.create_service(GetMessageInfo,
                                                        '/fsw_ros2_bridge/get_message_info',
                                                        self.get_message_info_callback)
        self.set_message_info_srv = self.create_service(SetMessageInfo,
                                                        '/fsw_ros2_bridge/set_message_info',
                                                        self.set_message_info_callback)
        self.get_plugin_info_srv = self.create_service(GetPluginInfo,
                                                       '/fsw_ros2_bridge/get_plugin_info',
                                                       self.get_plugin_info_callback)

        self.get_logger().warn("msg package: " + self._msg_pkg)

        self._timer_period = 0.1  # seconds

        self._telem_info = self._fsw.get_plugin().get_telemetry_message_info()
        self._command_info = self._fsw.get_plugin().get_command_message_info()

        self._pub_map = {}
        if self._telem_info:
            for t in self._telem_info:
                self.get_logger().warn("setting up pub with key:: " + t.get_key())
                self._pub_map[t.get_key()] = self.setup_publisher(t.get_key(),
                                                                  t.get_msg_type(),
                                                                  t.get_topic_name())

        self.sub_map = {}
        if self._command_info:
            for c in self._command_info:
                self.sub_map[c.get_key()] = self.create_subscriber(c.get_key(),
                                                                   c.get_msg_type(),
                                                                   c.get_topic_name(),
                                                                   c.get_callback_func())

        self._timer = self.create_timer(self._timer_period, self.timer_callback)
        self.load_message_info()

    def get_telemetry_message_types(self):
        """
        Gets the message types that are telemetry.

        Returns:
            telem_list (list): List of telemetry types
        """
        tt = []
        if self._telem_info:
            for t in self._telem_info:
                tt.append(t.get_msg_type())
        return tt

    def get_command_message_types(self):
        """
        Gets all the message types that are commands.

        Returns:
            command_list (list): List of command types
        """
        ct = []
        if self._command_info:
            for c in self._command_info:
                ct.append(c.get_msg_type())
        return ct

    def setup_publisher(self, key, msg_type, topic_name):
        """
        Set the publisher for the given telemetry type.

        Args:
            key (str): Unique telemetry identifier
            msg_type (str): The message type of the telemetry
            topic_name (str): The ROS2 topic name

        Returns:
            publisher: the created publisher
        """
        self.get_logger().info("Creating TLM (" + key + ") with name: " + topic_name
                               + " of type: " + msg_type)
        msg_path = self._msg_pkg + ".msg"
        msg_type = msg_type.replace(".msg", "")
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            return self.create_publisher(MsgType, topic_name, 10)
        except (AttributeError):
            self.get_logger().warn("Could not import TLM msg: " + msg_type)
            pass

    def create_subscriber(self, key, msg_type, topic_name, callback_func):
        """
        Set the subscriber for the given command type.

        Args:
            key (str): Unique command identifier
            msg_type (str): The message type of the command
            topic_name (str): The ROS2 topic name
            callback_func (func): The function to be called when the command is received
        """
        msg_path = self._msg_pkg + ".msg"
        msg_type = msg_type.replace(".msg", "")
        self.get_logger().info("Creating CMD (" + key + ") with name: " + topic_name
                               + " of type: " + msg_type)
        # self.get_logger().info("msg_path: " + msg_path)
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            self.subscription = self.create_subscription(MsgType, topic_name, callback_func, 10)
        except (AttributeError):
            self.get_logger().warn("... could not import CMD msg: " + msg_type)
            pass

    def timer_callback(self):
        """
        Callback to check for updated telemetry.
        """
        if self._telem_info:
            for t in self._telem_info:
                key = t.get_key()
                msgs = self._fsw.get_plugin().get_buffered_data(key, True)
                if msgs is None:
                    continue
                for msg in msgs:
                    # self.get_logger().info("[" + key + "] got data. ready to publish")
                    try:
                        msg.header.stamp = self.get_clock().now().to_msg()
                    except (AttributeError):
                        pass

                    self._pub_map[key].publish(msg)

    def load_message_info(self):
        """
        Loads ROS2 message structures on initialization.
        """
        self._message_info = []
        for package_name, message_names in get_message_interfaces().items():
            if package_name == self._msg_pkg:
                for message_name in message_names:
                    m = f'{package_name}/{message_name}'
                    self.get_logger().debug("found msg type: " + m + ", pkg: "
                                            + package_name + ", msg: " + message_name)
                    message_name = message_name.replace("msg/", "")
                    MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                                      message_name)
                    mi = MessageInfo()
                    mi.pkg_name = self._msg_pkg
                    mi.msg_name = message_name
                    mi.json = str(MsgType.get_fields_and_field_types())
                    # self.get_logger().info(mi.yaml)

                    if message_name in self.get_telemetry_message_types():
                        mi.msg_type = MessageInfo.TELEMETRY
                    elif message_name in self.get_command_message_types():
                        mi.msg_type = MessageInfo.COMMAND
                    else:
                        mi.msg_type = -1

                    self._message_info.append(mi)

        self.load_message_dictionary()

    def get_message_info_callback(self, request, response):
        """
        Callback that returns the message info list.

        Args:
            request: not used
            response: used for return value

        Returns:
            response: Value containing the message info list
        """
        self.get_logger().info('GetMessageInfo()')
        response.msg_info = self._message_info
        return response

    def set_message_info_callback(self, request, response):
        """
        Callback that sets the message info list.

        Args:
            request: contains the new message info
            response: not used

        Returns:
            response: not modified here
        """
        self.get_logger().info('SetMessageInfo()')
        for mi in self._message_info:
            if (mi.msg_name == request.msg_info.msg_name) and \
               (mi.pkg_name == request.msg_info.pkg_name):
                mi.info = request.msg_info.info
                self._msg_dict[mi.msg_name]["info"] = request.msg_info.info
                self.get_logger().info('SetMessageInfo() -- msg: ' + mi.msg_name)
                self.get_logger().info('SetMessageInfo() --  mi info  : ' + mi.info)
                self.get_logger().info('SetMessageInfo() --  dict info: '
                                       + self._msg_dict[mi.msg_name]["info"])
                self.save_msg_dict_to_disk()
        return response

    def get_plugin_info_callback(self, request, response):
        """
        Callback that returns plugin information.

        Args:
            request: not used
            response: not used

        Returns:
            response: Information about the plugin
        """
        response.node_name = self.get_name()
        response.msg_pkg = self._msg_pkg
        response.plugin_name = self._plugin_name
        response.config_files = self.get_config_files()
        return response

    def get_config_files(self):
        """
        Get the configuration files.

        Returns:
            config_files (list): Returns a list of yaml config files.
        """
        resource_path = get_package_share_directory(self._plugin_pkg_name) + r"/config/*.yaml"
        config_files = []
        config_files = glob.glob(resource_path)
        self.get_logger().info('get_config_files() -- ' + str(config_files))
        return config_files

    def load_message_dictionary(self):
        """
        Loads the message dictionary upon initialization.
        """
        resource_path = get_package_share_directory(self._msg_pkg)
        dict_file = 'message_dictionary.json'
        self._dict_file = resource_path + "/resource/" + dict_file
        self.get_logger().info("load_message_dictionary() -- dictionary file: " + self._dict_file)

        self._msg_dict = {}
        try:
            self.get_logger().info("opening message dictionary file....")
            with open(self._dict_file, "r") as infile:
                self._msg_dict = json.load(infile)
            self.get_logger().info("found message dictionary of size: "
                                   + str(len(self._msg_dict.keys())))
        except FileNotFoundError:
            self.get_logger().info("problem reading message dictionary")

        if not self._msg_dict:
            self.get_logger().info("need to create message dictionary")
            self._msg_dict = self.create_message_dictionary()
            self.save_msg_dict_to_disk()
        else:
            self.get_logger().info("found message dictionary...")
            self.copy_message_dictonary_to_info()

    def copy_message_dictonary_to_info(self):
        """
        Updates the message dictionary based on the current message information list.
        """
        self.get_logger().info("copy_message_dictonary_to_info() -- msg info size: "
                               + str(len(self._message_info)))
        modified = False
        for mi in self._message_info:
            if mi.msg_name in self._msg_dict:
                mi.info = self._msg_dict[mi.msg_name]["info"]
            else:
                self._msg_dict[mi.msg_name] = {"pkg": mi.pkg_name,
                                               "name": mi.msg_name,
                                               "type": self.get_message_type(mi.msg_type),
                                               "info": mi.info}
                modified = modified or True
        if modified:
            self.save_msg_dict_to_disk()

    def create_message_dictionary(self):
        """
        Creates the message dictionary from the current message information list.

        Returns:
            md (dict): the newly created message dictionary
        """
        md = {}
        for mi in self._message_info:
            mi.info = str("This is info about " + self.get_message_type(mi.msg_type)
                          + " msg: " + mi.msg_name)
            m = {"pkg": mi.pkg_name,
                 "name": mi.msg_name,
                 "type": self.get_message_type(mi.msg_type),
                 "info": mi.info}
            md[mi.msg_name] = m
        return md

    def save_msg_dict_to_disk(self):
        """
        Saves the current message dictionary to a file.
        """
        self.get_logger().info("saving to file: " + self._dict_file)
        msg_resource_path = get_package_share_directory(self._msg_pkg) + "/resource"
        self.get_logger().info("checking if path exists: " + msg_resource_path)

        if not os.path.exists(msg_resource_path):
            self.get_logger().info("creating path: " + msg_resource_path)
            os.makedirs(msg_resource_path)
        with open(self._dict_file, "w") as outfile:
            json.dump(self._msg_dict, outfile)

    def get_message_type(self, msg_type):
        """
        Returns a string specifying the message type.

        Args:
            msg_type: The message type to convert to a string

        Returns:
            type (str): The type of the message (telemetry or command)
        """
        if msg_type is MessageInfo.TELEMETRY:
            return "TELEMETRY"
        if msg_type is MessageInfo.COMMAND:
            return "COMMAND"
        else:
            return "UNKNOWN"


def main(args=None):
    rclpy.init(args=args)

    bridge = FSWBridge()
    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
