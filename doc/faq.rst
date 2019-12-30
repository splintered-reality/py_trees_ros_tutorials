.. _faq-section-label:

FAQ
===

ROS related frequently asked questions.

.. seealso:: The :ref:`py_trees:faq-section-label` from the py_trees package.

Parameter/Remap Proliferation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can imagine once you have 50+ re-usable behaviours in a tree
that the need for remapping of topics, services and parameters in the behaviour tree
launch description will become exceedingly large. In these situations it is more convenient
to load parameters for these remappings in a structured way on the parameter server
(loaded from a single yaml). This centralises your application configuration and additionally
exposes that configuration at runtime which will assist with debugging. Sanity...

On the parameter server, such configuration might look like:

.. code-block:: python

   /tree/topics/odom                 /gopher/odom
   /tree/topics/pose                 /gopher/pose
   /tree/services/get_global_costmap /move_base/global/get_costmap
   /tree/parameters/max_speed        /trajectory_controller/max_speed

In code, highlighting re-usability of the remappings across multiple behaviours:

.. code-block:: python

   odometry_topic=self.node.get_parameter_or(name="~topics/odom", alternative_value="/odom")
   pose_topic=self.node.get_parameter(name="~topics/pose", alternative_value="/pose")
   move_base = my_behaviours.MoveBaseClient(odometry_topic, pose_topic)
   odometry_foo = my_behvaiours.OdometryFoo(odometry_topic)


Continuous Tick-Tock?
^^^^^^^^^^^^^^^^^^^^^

Even though the behaviour tree provides a continuous tick-tock method, 
you can set your own pace. This can be useful if you wish to vary the tick
duration, or to tick only when an external trigger is received (a common
trick to minimise cpu usage in games). For example:

.. code-block:: python

   ...
   while rclpy.ok():
       rclpy.spin_once(timeout_sec=0.1)
       if some_external_trigger:
           tree.tick_once()

Triggering based on logic inside the tree however, is much more challenging
as this is almost a chicken and egg situation (tick only when an event
fires, but events are typically embedded in the decision making tree itself).
UE4 has an implementation that has crafted mechanisms for this which go beyond
basic behaviour tree concepts - if you have such a need, it's likely you'll
have to extend py_trees to meet the needs of your own use case.

Control-Level Decision Making
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Our first use case never intended to utilise behaviour trees for decision making
typically considered internal to control subsystems. A good example of such is the
approach logic for a docking maneuvre. Another is the recovery behaviours for
navigation, which start to access sound/light notifications, specialised sensing
contexts as well as specialised maneuvres. Note that neither of these require
low-latency for their decision logic. Nonetheless, it was surprising
to find the control engineers moving the logic from internal state machines to
the behaviour trees at a higher level.

In hindsight, this makes good sense. With the robot's decision making logic
landing in one place, logging, debugging and visualising the state of the robot
became simpler and could make use of a single set of tools. A growing library
of shared and reusable patterns sped up the development cycle.
It also liberated subsystems from having to co-ordinate other subsystems (e.g. the
navigation system when engaging in recovery behaviours).     
