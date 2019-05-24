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
import py_trees_ros
import rcl_interfaces.msg as rcl_msgs
import rcl_interfaces.srv as rcl_srvs
import rclpy
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

        self.cached_context = None

    def setup(self, **kwargs):
        """
        Setup the service clients for getting and setting the context (parameter services
        for dynamic parameter configuration).

        Args:
            **kwargs (:obj:`dict`): look for the 'node' object being passed down from the tree

        Raises:
            :class:`KeyError`: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        self.logger.debug("%s.setup()" % self.__class__.__name__)

        # ros2 node
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        # parameter service clients
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
        for name, client in self.parameter_clients.items():
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError("client timed out waiting for server [{}]".format(name))

    def initialise(self):
        """
        Get various dyn reconf configurations and cache/set the new variables.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        self.cached_context = None
        # kickstart get/set parameter chain
        self.send_get_parameter_request()

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        all_done = False

        # wait for get_parameter to return
        if self.cached_context is None:
            if self.process_get_parameter_response():
                self.send_set_parameter_request(value=True)
            return py_trees.common.Status.RUNNING

        # wait for set parameter to return
        if not all_done:
            if self.process_set_parameter_response():
                all_done = True
            return py_trees.common.Status.RUNNING

        # just spin around, wait for an interrupt to trigger terminate
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Regardless of whether it succeeed or failed or is getting set to invalid we have to be absolutely
        sure to reset the navi context.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if (
            new_status == py_trees.common.Status.INVALID and
            self.cached_context is not None
           ):
            self.send_set_parameter_request(value=self.cached_context)
            # don't worry about the response, no chance to catch it anyway

    def send_get_parameter_request(self):
        request = rcl_srvs.GetParameters.Request()  # noqa
        request.names.append("enabled")
        self.get_parameter_future = self.parameter_clients['get_safety_sensors'].call_async(request)

    def process_get_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.get_parameter_future.result() is None:
            self.feedback_message = "failed to retrieve the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('Service call failed %r' % (future.exception(),))
            raise RuntimeError(self.feedback_message)
        if len(self.get_parameter_future.result().values) > 1:
            self.feedback_message = "expected one parameter value, got multiple [{}]".format("/safety_sensors/enabled")
            raise RuntimeError(self.feedback_message)
        value = self.get_parameter_future.result().values[0]
        if value.type != rcl_msgs.ParameterType.PARAMETER_BOOL:  # noqa
            self.feedback_message = "expected parameter type bool, got [{}]{}]".format(value.type, "/safety_sensors/enabled")
            self.node.get_logger().error(self.feedback_message)
            raise RuntimeError(self.feedback_message)
        self.cached_context = value.bool_value
        return True

    def send_set_parameter_request(self, value: bool):
        request = rcl_srvs.SetParameters.Request()  # noqa
        parameter = rcl_msgs.Parameter()
        parameter.name = "enabled"
        parameter.value.type = rcl_msgs.ParameterType.PARAMETER_BOOL  # noqa
        parameter.value.bool_value = value
        request.parameters.append(parameter)
        self.set_parameter_future = self.parameter_clients['set_safety_sensors'].call_async(request)

    def process_set_parameter_response(self) -> bool:
        if not self.get_parameter_future.done():
            return False
        if self.set_parameter_future.result() is not None:
            self.feedback_message = "reconfigured the safety sensors context"
        else:
            self.feedback_message = "failed to reconfigure the safety sensors context"
            self.node.get_logger().error(self.feedback_message)
            # self.node.get_logger().info('service call failed %r' % (future.exception(),))
        return True
