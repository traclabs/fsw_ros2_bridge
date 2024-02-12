"""
.. module:: fsw_ros2_bridge.command_info
   :synopsis: Defines data structure used to define commands.

.. moduleauthor:: Stephen Hart

"""


class CommandInfo():
    """
    This class defines the command information needed for each command.

    Attributes
    ----------
    key : str
        a string that uniquely identifies this command
    msg_type : str
        the data type of the data passed with the command
    topic_name : str
        the ROS2 topic for the command
    callback_func : function
        the function that will be called when a command is received from ROS2
    port : int
        the port that the command will be sent over by the bridge

    Methods
    -------
    get_key()
        Gets the unique identifier for this command.
    get_msg_type()
        Gets the message type for this command.
    get_topic_name()
        Gets the ROS2 topic name for this command.
    get_callback_func()
        Gets the callback function for this command.
    set_callback_func(func)
        Sets the value of callback_function to 'func'
    get_port()
        Gets the port for this command.
    """

    def __init__(self, key, msg_type, topic_name, callback_func, port):
        """
        Initialize the attributes for the CommandInfo object.

        Args:
            key (str): A unique identifier for the command
            msg_type (str): The data type passed with the command
            topic_name (str): The ROS2 topic name to receive the command from
            callback_func (function): The function to be called when a command is received
            port (int): The port number to send the command over
        """
        self._key = key
        self._msg_type = msg_type
        self._topic_name = topic_name
        self._callback_func = callback_func
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

    def get_callback_func(self):
        """
        Gets the callback function for this command.

        Returns:
            callback_func (function): the callback function for this command
        """
        return self._callback_func

    def set_callback_func(self, func):
        """
        Sets the callback function for this command.

        Args:
            func (function): the function to be called when a command is received
        """
        self._callback_func = func

    def get_port(self):
        """
        Gets the port for this command.

        Returns:
            port (int): the port for this command
        """
        return self._port
