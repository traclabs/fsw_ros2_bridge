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
   
    def __init__(self, plugin=None):
        super().__init__('fsw_ros2_bridge')

        if plugin:
            self.get_logger().warn("Using FSW Plugin: " + plugin)
        else:
            self.get_logger().warn("No FSW Plugin specified, using test")
        self.fsw = fsw_ros2_bridge.fsw_wrapper.FSWWrapper(plugin)
        # self.fsw = fsw_ros2_bridge.fsw_wrapper.FSWWrapper("cfs_groundsystem_bridge_plugin.cfs_groundsystem_bridge_plugin")
        # self.fsw = fsw_ros2_bridge.fsw_wrapper.FSWWrapper("fsw_ros2_bridge.juicer_bridge")

        self.timer_period = 0.5  # seconds

        self.telem_info = self.fsw.getPlugin().getTelemetryMessageInfo()
        self.command_info = self.fsw.getPlugin().getCommandMessageInfo()

        self.pub_map = {}
        if self.telem_info :
            for t in self.telem_info :
                self.pub_map[t.getKey()] = self.createPublisher(t.getMsgType(), t.getTopicName()) 

        self.sub_map = {}
        if self.command_info :
            for c in self.command_info :
                self.sub_map[c.getKey()] = self.createSubscriber(c.getKey(), c.getTopicName(), c.getCallbackFunc()) 

        self.timer = self.create_timer(self.timer_period, self.timerCallback)


    def createPublisher(self, msgType, topicName) :
        self.get_logger().info("Creating PUB topic with name: " + topicName + " of type: " + msgType)

        msg_path = self.fsw.getMsgPackage() + ".msg"
        #MsgType = getattr(importlib.import_module("cfs_msgs.msg"), msgType)  
        MsgType = getattr(importlib.import_module(msg_path), msgType)  
        return self.create_publisher(MsgType, topicName, 10)

    def createSubscriber(self, key, topicName, callbackFunc) :
        self.get_logger().warn("Creating CMD topic with name: " + topicName)
        # self.subscription = self.create_subscription(ToLabCmd, topicName, callbackFunc, 10)

    def timerCallback(self):
        if self.telem_info :
            for t in self.telem_info:
                key = t.getKey()
                msg = self.fsw.getPlugin().getLatestData(key)
                if msg != None:
                    self.get_logger().debug("[" + key + "] got data. ready to publish")
                    self.pub_map[key].publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # parse args and pass in plugin
    bridge = FSWBridge()
    rclpy.spin(bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
