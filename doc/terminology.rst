Terminology
===========

.. glossary::

   block
   blocking
      A behaviour is sometimes referred to as a 'blocking' behaviour. Technically, the execution
      of a behaviour should be non-blocking (i.e. the tick part), however when it's progress from
      'RUNNING' to 'FAILURE/SUCCESS' takes more than one tick, we say that the behaviour itself
      is blocking. In short, `blocking == RUNNING`.

   context switch
      Very often a task or sequence of tasks will require a context switch of
      the runtime system. For example, enabling additional sensing and
      processing pipelines in order to navigate a staircase. The context switch is
      typically the modification of a few dynamic parameters and/or service calls
      to configure runtime nodes to behave in a different manner. The key
      requirements for a context switch is to cache the original context,
      hold the configuration throughout the context and then reset the context to
      the cached state upon completion. This falls naturally into a behaviour's
      :meth:`~py_trees.behaviour.Behaviour.initialise()`,
      :meth:`~py_trees.behaviour.Behaviour.update()` and
      :meth:`~py_trees.behaviour.Behaviour.terminate()` modalities. To ensure
      it activates at the appropriate time, drop it into a parallel alongside
      the activity that requires the context switch.

      .. seealso:: :ref:`tutorial-six`

   data gathering
      Caching events, notifications, or incoming data arriving asynchronously on the blackboard.
      This is a fairly common practice for behaviour trees which exist inside a complex system.
      In the ROS world, it is most likely you will catch data coming in on subscribers in this way.

      In most cases, data gathering is done at the front end of your tree under a parallel
      directly alongside your priority work selector.

      .. seealso:: :ref:`tutorial-one`

   mock
   mocking
      A very useful paradigm to accelerate development and testing of your behaviour trees
      is to mock your robot with very simple stubs that provide the same ROS API as the real
      robot. The actual behaviour underneath that ROS API need only be very roughly connected
      to the real thing.

      .. note:: The key here is to test the decision making in your behaviour tree.

      In most cases this has very little to do with the kinematics, dynamics or sensor
      fidelity of a full simulation.

      Mocking the bits and pieces takes far less time and you'll also be able to
      insert handles that can help you force decision making to
      branch to where you want to test. For example, using dynamic reconfigure in
      :class:`py_trees_ros.mock.battery.Battery` to abruptly force charging/discharging and at
      varying rates. Additionally, if you set up the mock well, you'll find it executes far faster
      than a full simulation (you can montage - no need to endure travel time).

      t will also make your web team happier (for apps that sit astride the behaiviour tree).
      These apps typically require thorough testing of decision making branches that are not
      often traversed e.g. battery low recovery handling, or cancelling procedures.
      This is far easier to do in a mock. They'll also appreciate not having to setup the
      entire infrastructure necessary for a dynamic simulation.
