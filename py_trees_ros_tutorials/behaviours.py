#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees_ros_tutorials/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviours for the tutorials.
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import py_trees_ros
import rcl_interfaces.srv as rcl_srvs
import std_msgs.msg as std_msgs

##############################################################################
# Behaviours
##############################################################################


class FlashLedStrip(py_trees.behaviour.Behaviour):
    """
    This behaviour simply shoots a command off to the LEDStrip to flash
    a certain colour and returns :attr:`~py_trees.common.Status.RUNNING`.
    Note that this behaviour will never return with
    :attr:`~py_trees.common.Status.SUCCESS` but will send a clearing
    command to the LEDStrip if it is cancelled or interrupted by a higher
    priority behaviour.

    Publishers:
        * **/led_strip/command** (:class:`std_msgs.msg.String`)

          * colourised string command for the led strip ['red', 'green', 'blue']

    Args:
        name: name of the behaviour
        topic_name : name of the battery state topic
        colour: colour to flash ['red', 'green', blue']
    """
    def __init__(
            self,
            name: str,
            topic_name: str="/led_strip/command",
            colour: str="red"
         ):
        super(FlashLedStrip, self).__init__(name=name)
        self.topic_name = topic_name
        self.colour = colour

    def setup(self, **kwargs):
        """
        Setup the publisher which will stream commands to the mock robot.

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.publisher = self.node.create_publisher(
            msg_type=std_msgs.String,
            topic=self.topic_name,
            qos_profile=py_trees_ros.utilities.qos_profile_latched_topic()
        )
        self.feedback_message = "publisher created"

    def update(self) -> py_trees.common.Status:
        """
        Annoy the led strip to keep firing every time it ticks over (the led strip will clear itself
        if no command is forthcoming within a certain period of time).
        This behaviour will only finish if it is terminated or priority interrupted from above.

        Returns:
            Always returns :attr:`~py_trees.common.Status.RUNNING`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.publisher.publish(std_msgs.String(data=self.colour))
        self.feedback_message = "flashing {0}".format(self.colour)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Shoot off a clearing command to the led strip.

        Args:
            new_status: the behaviour is transitioning to this new status
        """
        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                "{}->{}".format(self.status, new_status) if self.status != new_status else "{}".format(new_status)
            )
        )
        self.publisher.publish(std_msgs.String(data=""))
        self.feedback_message = "cleared"


class ScanContext(py_trees.behaviour.Behaviour):
    """
    <TODO>

    Args:
        name (:obj:`str`): name of the behaviour
    """
    def __init__(self, name):
        super().__init__(name=name)

        self.initialised = False
        self._namespaces = ["safety_sensors",
                            "rotate",
                            ]
        self._dynamic_reconfigure_clients = {}
        for name in self._namespaces:
            self._dynamic_reconfigure_clients[name] = None
        self._dynamic_reconfigure_configurations = {}

    def setup(self, timeout):
        """
        Try and connect to the dynamic reconfigure server on the various namespaces.
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)
        self.parameter_clients = {
            'get_safety_sensors': self.node.create_client(
                rcl_srvs.GetParameters,
                '/safety_sensors/get_parameters'
            ),
            'set_safety_sensors': self.node.create_client(
                rcl_srvs.SetParameters,
                '/safety_sensors/set_parameters'
            )
        }

    def initialise(self):
        """
        Get various dyn reconf configurations and cache/set the new variables.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

        for name, client in self._dynamic_reconfigure_clients.iteritems():
            self._dynamic_reconfigure_configurations[name] = client.get_configuration()
        try:
            self.safety_sensors_enable = self._dynamic_reconfigure_configurations["safety_sensors"]["enable"]
            self._dynamic_reconfigure_clients["safety_sensors"].update_configuration({"enable": True})
        except dynamic_reconfigure.DynamicReconfigureParameterException:
            self.feedback_message = "failed to configure the 'enable' parameter [safety_sensors]"
            self.initialised = False
        try:
            self.rotate_duration = self._dynamic_reconfigure_configurations["rotate"]["duration"]
            self._dynamic_reconfigure_clients["rotate"].update_configuration({"duration": 8.0})
        except dynamic_reconfigure.DynamicReconfigureParameterException:
            self.feedback_message = "failed to configure the 'duration' parameter [rotate]"
            self.initialised = False

        self.initialised = True
        self.feedback_message = "reconfigured the context for scanning"

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if not self.initialised:
            return py_trees.common.Status.FAILURE
        # used under a parallel, never returns success
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Regardless of whether it succeeed or failed or is getting set to invalid we have to be absolutely
        sure to reset the navi context.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.initialised:
            try:
                self._dynamic_reconfigure_clients["safety_sensors"].update_configuration({"enable": self.safety_sensors_enable})
            except dynamic_reconfigure.DynamicReconfigureParameterException:
                self.feedback_message = "failed to reset the 'enable' parameter [safety_sensors]"
            try:
                self._dynamic_reconfigure_clients["rotate"].update_configuration({"duration": self.rotate_duration})
            except dynamic_reconfigure.DynamicReconfigureParameterException:
                self.feedback_message = "failed to reset the 'duration' parameter [rotate]"
            self.initialised = False
