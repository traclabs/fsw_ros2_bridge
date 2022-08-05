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
        self.msg_pkg = self.fsw.getMsgPackage()

        self.get_logger().warn("msg package: " + self.msg_pkg)

        self.timer_period = 0.5  # seconds

        self.telem_info = self.fsw.getPlugin().getTelemetryMessageInfo()
        self.command_info = self.fsw.getPlugin().getCommandMessageInfo()

        self.pub_map = {}
        if self.telem_info:
            for t in self.telem_info:
                self.pub_map[t.getKey()] = self.createPublisher(t.getMsgType(), t.getTopicName())

        self.sub_map = {}
        if self.command_info:
            for c in self.command_info:
                self.sub_map[c.getKey()] = self.createSubscriber(c.getKey(), c.getMsgType(), c.getTopicName(), c.getCallbackFunc())

        self.timer = self.create_timer(self.timer_period, self.timerCallback)

    def createPublisher(self, msgType, topicName):
        self.get_logger().info("Creating PUB topic with name: " + topicName + " of type: " + msgType)
        msg_path = self.msg_pkg + ".msg"
        try :
            MsgType = getattr(importlib.import_module(msg_path), msgType)
            return self.create_publisher(MsgType, topicName, 10)
        except :
            self.get_logger().warn("Could not import TLM msg: " + msgType)
            pass

    def createSubscriber(self, key, msgType, topicName, callbackFunc):
        self.get_logger().info("Creating CMD topic with name: " + topicName + " of type: " + msgType)
        msg_path = self.msg_pkg + ".msg"
        try :
            MsgType = getattr(importlib.import_module(msg_path), msgType)
            self.subscription = self.create_subscription(MsgType, topicName, callbackFunc, 10)
        except :
            self.get_logger().warn("Could not import CMD msg: " + msgType)
            pass

    def timerCallback(self):
        if self.telem_info:
            for t in self.telem_info:
                key = t.getKey()
                msg = self.fsw.getPlugin().getLatestData(key)
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
