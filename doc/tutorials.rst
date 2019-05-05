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

