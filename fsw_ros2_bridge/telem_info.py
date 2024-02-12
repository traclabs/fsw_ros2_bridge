"""
.. module:: fsw_ros2_bridge.telem_info
   :synopsis: Defines data structure used to define telemetry.

.. moduleauthor:: Stephen Hart

"""


class TelemInfo():
    """
    This class defines the information needed for each telemetry item.

    Attributes
    ----------
    key : str
        a string that uniquely identifies this telemetry
    msg_type : str
        the data type of the data passed with the telemetry
    topic_name : str
        the ROS2 topic for the telemetry
    port : int
        the port that the telemetry will be received by the bridge

    Methods
    -------
    get_key()
        Gets the unique identifier for this command.
    get_msg_type()
        Gets the message type for this command.
    get_topic_name()
        Gets the ROS2 topic name for this command.
    get_port()
        Gets the port for this command.
    """

    def __init__(self, key, msg_type, topic_name, port):
        '''
        Initialize the attributes for the TelemInfo object.

        Args:
            key (str): A unique identifier for the telemetry
            msg_type (str): The data type passed with the telemetry
            topic_name (str): The ROS2 topic name for the telemetry
            port (int): The port number to receive the telemetry from
        '''
        self._key = key
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._port = port

    def get_key(self):
        """
        Gets the unique identifier for this command.

        Returns:
            key (str): the unique identifier for this command
        """
        return self._key

    def get_msg_type(self):
        """
        Gets the message type for this command.

        Returns:
            msg_type (str): the message type for this command
        """
        return self._msg_type

    def get_topic_name(self):
        """
        Gets the ROS2 topic name for this command.

        Returns:
            topic_name (str): the topic name for this command
        """
        return self._topic_name

    def get_port(self):
        """
        Gets the port for this command.

        Returns:
            port (int): the port for this command
        """
        return self._port
