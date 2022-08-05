#!/usr/bin/env python3

from abc import ABC, abstractmethod

from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo


class FSWPluginInterface:

    def __init__(self, node):
        self.telem_info = []
        self.command_info = []
        self.node = node

    @abstractmethod
    def getTelemetryMessageInfo(self):
        pass

    @abstractmethod
    def getCommandMessageInfo(self):
        pass

    @abstractmethod
    def getLatestData(self, key):
        pass

    @abstractmethod
    def createROSMsgs(self, msg_dir):
        pass

    @abstractmethod
    def getMsgPackage(self):
        pass
