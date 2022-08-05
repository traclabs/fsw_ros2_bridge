
class CommandInfo():

    def __init__(self, key, msgType, topicName, callbackFunc) :       
    # def __init__(self, key, msgType, topicName) :       
        self._key = key
        self._msgType = msgType
        self._topicName = topicName
        self._callbackFunc = callbackFunc

    def getKey(self) :
        return self._key
    
    def getMsgType(self) :
        return self._msgType

    def getTopicName(self) :
        return self._topicName
    
    def getCallbackFunc(self) :
        return self._callbackFunc

    def print(self) :
        print("key: " + self._key + ", msg: " + self._msgType + ", topic: " + self._topicName)

