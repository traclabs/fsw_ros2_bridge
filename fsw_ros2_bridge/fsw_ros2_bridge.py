import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory

import importlib
import json

import fsw_ros2_bridge.fsw_wrapper
from fsw_ros2_bridge_msgs.srv import GetMessageInfo
from fsw_ros2_bridge_msgs.srv import SetMessageInfo
from fsw_ros2_bridge_msgs.srv import GetPluginInfo
from fsw_ros2_bridge_msgs.msg import MessageInfo


class FSWBridge(Node):

    def __init__(self):
        super().__init__('fsw_ros2_bridge')

        self._dict_file = ""
        self._message_info = []
        self._msg_dict = {}

        # default plugin param, will be set/loaded from app config file
        self.declare_parameter('plugin_name', 'fsw_ros2_bridge.test_plugin')
        self._plugin_name = self.get_parameter('plugin_name').get_parameter_value().string_value

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

        self._timer_period = 0.5  # seconds

        self._telem_info = self._fsw.get_plugin().get_telemetry_message_info()
        self._command_info = self._fsw.get_plugin().get_command_message_info()

        self._pub_map = {}
        if self._telem_info:
            for t in self._telem_info:
                self._pub_map[t.get_key()] = self.setup_publisher(t.get_msg_type(),
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
        tt = []
        if self._telem_info:
            for t in self._telem_info:
                tt.append(t.get_msg_type())
        return tt

    def get_command_message_types(self):
        ct = []
        if self._command_info:
            for c in self._command_info:
                ct.append(c.get_msg_type())
        return ct

    def setup_publisher(self, msg_type, topic_name):
        self.get_logger().info("Creating PUB with name: " + topic_name + " of type: " + msg_type)
        msg_path = self._msg_pkg + ".msg"
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            return self.create_publisher(MsgType, topic_name, 10)
        except (AttributeError):
            self.get_logger().warn("Could not import TLM msg: " + msg_type)
            pass

    def create_subscriber(self, key, msg_type, topic_name, callback_func):
        self.get_logger().info("Creating CMD with name: " + topic_name + " of type: " + msg_type)
        msg_path = self._msg_pkg + ".msg"
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            self.subscription = self.create_subscription(MsgType, topic_name, callback_func, 10)
        except (AttributeError):
            self.get_logger().warn("Could not import CMD msg: " + msg_type)
            pass

    def timer_callback(self):
        if self._telem_info:
            for t in self._telem_info:
                key = t.get_key()
                msg = self._fsw.get_plugin().get_latest_data(key)
                if msg is not None:
                    self.get_logger().debug("[" + key + "] got data. ready to publish")
                    self._pub_map[key].publish(msg)

    def load_message_info(self):
        self._message_info = []
        for package_name, message_names in get_message_interfaces().items():
            if package_name == self._msg_pkg:
                for message_name in message_names:
                    m = f'{package_name}/{message_name}'
                    self.get_logger().info("found msg type: " + m + ", pkg: "
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
        self.get_logger().info('GetMessageInfo()')
        response.msg_info = self._message_info
        return response

    def set_message_info_callback(self, request, response):
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
        response.msg_pkg = self._msg_pkg
        response.plugin_name = self._plugin_name
        return response

    def load_message_dictionary(self):
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
        self.get_logger().info("saving to file: " + self._dict_file)
        with open(self._dict_file, "w") as outfile:
            json.dump(self._msg_dict, outfile)

    def get_message_type(self, msg_type):
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
