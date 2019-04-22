#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import action_msgs.msg as action_msgs  # GoalStatus
import py_trees
import py_trees.console as console
import rclpy
import rclpy.executors
import time
import unittest

import py_trees_ros_tutorials.mock as mock

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (40 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)

##############################################################################
# Tests
##############################################################################


class TestActionServers(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        console.banner("ROS Init")
        rclpy.init()
        cls.timeout = 3.0
        cls.blackboard = py_trees.blackboard.Blackboard()

    @classmethod
    def tearDownClass(cls):
        console.banner("ROS Shutdown")
        rclpy.shutdown()

    def setUp(self):
        pass

##############################################################################
# Success
##############################################################################

    def generic_success_test(
            self,
            title,
            server,
            client
         ):
        console.banner(title)

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(client.node)

        # Send goal and await future
        client.setup()
        goal_future = client.send_goal()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if goal_future.result() is not None:
                break
        assert_banner()
        assert_details("goal_future.result()", "!None", goal_future.result())
        self.assertTrue(goal_future.result() is not None)
        print("goal_future.result().accepted.....True [{}]".format(goal_future.result().accepted))
        self.assertTrue(goal_future.result().accepted)
        goal_handle = goal_future.result()

        # Await goal result future
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if result_future.done():
                break

        assert_banner()
        assert_details("result_future.done()", "True", result_future.done())
        self.assertTrue(result_future.done())
        assert_details(
            "result_future.result().action_status",
            "STATUS_SUCCEEDED",
            client.status_strings[result_future.result().action_status]
        )
        self.assertTrue(result_future.result().action_status ==
                        action_msgs.GoalStatus.STATUS_SUCCEEDED)  # noqa
        executor.shutdown()
        server.shutdown()
        client.shutdown()

    def test_move_base_success(self):
        self.generic_success_test(
            title="MoveBase Success",
            server=mock.move_base.MoveBase(duration=0.5),
            client=mock.actions.MoveBaseClient())

    def test_dock_success(self):
        self.generic_success_test(
            title="Dock Success",
            server=mock.dock.Dock(duration=0.5),
            client=mock.actions.DockClient())

    def test_rotate_success(self):
        self.generic_success_test(
            title="Rotate Success",
            server=mock.rotate.Rotate(rotation_rate=3.14),
            client=mock.actions.RotateClient())

##############################################################################
# Preemption
##############################################################################

    def generic_preemption_test(
            self,
            title,
            server,
            client
         ):
        console.banner(title)

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(client.node)

        # Send goal and await future
        client.setup()
        goal_future = client.send_goal()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if goal_future.result() is not None:
                break
        assert_banner()
        assert_details("goal_future.result()", "!None", goal_future.result())
        self.assertTrue(goal_future.result() is not None)
        print("goal_future.result().accepted.....True [{}]".format(goal_future.result().accepted))
        self.assertTrue(goal_future.result().accepted)
        goal_handle = goal_future.result()

        # preempt with another goal
        next_goal_future = client.action_client.send_goal_async(client.action_type.Goal())

        # Await preempted goal result future
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if result_future.done():
                break

        assert_banner()
        assert_details("result_future.done()", "True", result_future.done())
        self.assertTrue(result_future.done())
        assert_details(
            "result_future.result().action_status",
            "STATUS_ABORTED",
            client.status_strings[result_future.result().action_status]
        )
        self.assertTrue(result_future.result().action_status ==
                        action_msgs.GoalStatus.STATUS_ABORTED)  # noqa

        # Somewhat uncertain how this shutdown works.
        # The action server shutdown calls action_server.destroy()
        # which destroys goal handles and removes the action server
        # as a waitable on the node queue, however....there still
        # exists unexplicable mysteries
        #
        # - if executor.shutdown() is first, why doesn't it wait?
        # - why does the action server itself hang around till the
        #   execute() task is done/aborted instead of crashing?
        executor.shutdown()
        server.shutdown()
        client.shutdown()

    def test_dock_preemption(self):
        self.generic_preemption_test(
            title="Dock Preemption",
            server=mock.dock.Dock(duration=0.5),
            client=mock.actions.DockClient())

##############################################################################
# Cancel
##############################################################################

    def generic_cancel_test(
            self,
            title,
            server,
            client
         ):
        console.banner(title)

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(server.node)
        executor.add_node(client.node)

        # Send goal and await future
        client.setup()
        goal_future = client.send_goal()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if goal_future.result() is not None:
                break
        assert_banner()
        assert_details("goal_future.result()", "!None", goal_future.result())
        self.assertTrue(goal_future.result() is not None)
        print("goal_future.result().accepted.....True [{}]".format(goal_future.result().accepted))
        self.assertTrue(goal_future.result().accepted)
        goal_handle = goal_future.result()

        # cancel
        self._timer = client.node.create_timer(0.2, client.send_cancel_request)

        # Await preempted goal result future
        result_future = goal_handle.get_result_async()
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < self.timeout:
            executor.spin_once(timeout_sec=0.1)
            if result_future.done():
                break

        assert_banner()
        assert_details("result_future.done()", "True", result_future.done())
        self.assertTrue(result_future.done())
        assert_details(
            "result_future.result().action_status",
            "STATUS_CANCELED",
            client.status_strings[result_future.result().action_status]
        )
        self.assertTrue(result_future.result().action_status ==
                        action_msgs.GoalStatus.STATUS_CANCELED)  # noqa

        executor.shutdown()
        server.shutdown()
        client.shutdown()

    def test_dock_cancel(self):
        self.generic_cancel_test(
            title="Dock Cancel",
            server=mock.dock.Dock(duration=0.5),
            client=mock.actions.DockClient())


if __name__ == '__main__':
    unittest.main()
