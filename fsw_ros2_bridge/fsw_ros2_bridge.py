import rclpy
from rclpy.node import Node

from pathlib import Path
import importlib

import fsw_ros2_bridge.fsw_wrapper

from fsw_ros2_bridge.bcolors import bcolors
from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo

# from cfs_msgs.msg import ToLabCmd

class FSWBridge(Node):

    def __init__(self):
        super().__init__('fsw_ros2_bridge')

        # default plugin param, will be set/loaded from app config file
        self.declare_parameter('plugin_name', 'fsw_ros2_bridge.test_plugin')
        self.plugin_name = self.get_parameter('plugin_name').get_parameter_value().string_value

        self.get_logger().warn("============================================================================")
        self.get_logger().warn("Using FSW Plugin: " + self.plugin_name)
        self.get_logger().warn("============================================================================")
        self.fsw = fsw_ros2_bridge.fsw_wrapper.FSWWrapper(self, self.plugin_name)
        self.msg_pkg = self.fsw.get_msg_package()

        self.get_logger().warn("msg package: " + self.msg_pkg)

        self.timer_period = 0.5  # seconds

        self.telem_info = self.fsw.get_plugin().get_telemetry_message_info()
        self.command_info = self.fsw.get_plugin().get_command_message_info()

        self.pub_map = {}
        if self.telem_info:
            for t in self.telem_info:
                self.pub_map[t.get_key()] = self.setup_publisher(t.get_msg_type(), t.get_topic_name())

        self.sub_map = {}
        if self.command_info:
            for c in self.command_info:
                self.sub_map[c.get_key()] = self.create_subscriber(c.get_key(), c.get_msg_type(), c.get_topic_name(), c.get_callback_func())

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def setup_publisher(self, msg_type, topic_name):
        self.get_logger().info("Creating PUB topic with name: " + topic_name + " of type: " + msg_type)
        msg_path = self.msg_pkg + ".msg"
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            return self.create_publisher(MsgType, topic_name, 10)
        except:
            self.get_logger().warn("Could not import TLM msg: " + msg_type)
            pass

    def create_subscriber(self, key, msg_type, topic_name, callback_func):
        self.get_logger().info("Creating CMD topic with name: " + topic_name + " of type: " + msg_type)
        msg_path = self.msg_pkg + ".msg"
        try:
            MsgType = getattr(importlib.import_module(msg_path), msg_type)
            self.subscription = self.create_subscription(MsgType, topic_name, callback_func, 10)
        except:
            self.get_logger().warn("Could not import CMD msg: " + msg_type)
            pass

    def timer_callback(self):
        if self.telem_info:
            for t in self.telem_info:
                key = t.get_key()
                msg = self.fsw.get_plugin().get_latest_data(key)
                if msg != None:
                    self.get_logger().debug("[" + key + "] got data. ready to publish")
                    self.pub_map[key].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bridge = FSWBridge()
    rclpy.spin(bridge)

    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
