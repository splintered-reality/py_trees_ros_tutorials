.. _tutorials-section:

Tutorials
=========

The Mock Robot
--------------

The tutorials here all run atop a very simple :term:`mock` robot that
encapsulates the following list of mocked components:

* Battery
* LED Strip
* Docking Action Server
* Move Base Action Server
* Rotation Action Server
* Safety Sensors Pipeline

The :term:`mock` robot could just as easily be replaced by a gazebo
simulated robot or even real robot that implements exactly the same
ROS API interface.

The tutorials take care of launching the mock robot, but it can be also
launched on its own with:

.. code-block:: bash

    $ ros2 run py_trees_ros_tutorials mock-robot

.. _tutorial-one:

Tutorial 1 - Data Gathering
---------------------------

.. automodule:: py_trees_ros_tutorials.one_data_gathering
    :synopsis: data gathering with the battery to blackboard behaviour

.. _tutorial-two:

Tutorial 2 - Battery Check
--------------------------

.. automodule:: py_trees_ros_tutorials.two_battery_check
    :synopsis: adding a low battery check, with LED notification to the tree

.. _tutorial-three:

Tutorial 3 - Introspect the Blackboard
--------------------------------------

About
^^^^^

Tutorial three is a repeat of :ref:`tutorial-two`. The purpose of this
tutorial however is to introduce the tools provided to
allow introspection of the blackboard from ROS. Publishers and services
are provided by :class:`py_trees_ros.blackboard.Exchange`
which is embedded in a :class:`py_trees_ros.trees.BehaviourTree`. Interaction
with the exchange is over a set of services and dynamically created topics
via the the :ref:`py-trees-blackboard-watcher` command line utility.

Running
^^^^^^^

Launch the tutorial:

.. code-block:: bash

    # Launch the tutorial
    $ ros2 run py_trees_ros_tutorials tutorial-three-introspect-the-blackboard

In another shell:

.. code-block:: bash

    # check the entire board
    $ py-trees-blackboard-watcher
    # determine what you may stream
    $ py-trees-blackboard-watcher --list-variables
    # stream a simple variable (slide the battery level on the dashboard to trigger a change)
    $ py-trees-blackboard-watcher battery_low_warning
    # stream a nested variable
    $ py-trees-blackboard-watcher battery/percentage

.. image:: images/tutorial-three-introspect-the-blackboard.gif
