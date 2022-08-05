#!/usr/bin/env python3

from fsw_ros2_bridge.fsw_plugin_interface import *

class FSWPlugin(FSWPluginInterface):

    def getTelemetryMessageInfo(self) :
        pass

    def getCommandMessageInfo(self) :
        pass

    def getLatestData(self, key) :
        pass

    def getMsgPackage(self) :
        pass
