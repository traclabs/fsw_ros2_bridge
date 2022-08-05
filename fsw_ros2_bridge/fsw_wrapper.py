#!/usr/bin/env python3

import importlib
from fsw_ros2_bridge.fsw_plugin_interface import *

class FSWWrapper:
    # We are going to receive a plugin as parameter
    def __init__(self, plugin=None):
        if plugin != None :
            self._plugin = importlib.import_module(plugin,".").FSWPlugin()
        else :
            self._plugin = importlib.import_module("fsw_ros2_bridge.test_plugin", ".").FSWPlugin()

    def getPlugin(self) :
        return self._plugin

