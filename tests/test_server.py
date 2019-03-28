#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
##############################################################################
# Imports
##############################################################################

import rclpy
import rclpy.action
import rclpy.callback_groups
import rclpy.executors
import test_msgs.action as test_actions
import threading
import time

##############################################################################
# Fake
##############################################################################


class GenericServer(object):
    def __init__(self):
        self.node = rclpy.create_node("generic_server")
        self.goal_handle = None
        self.goal_lock = threading.Lock()

        self.action_server = rclpy.action.ActionServer(
            node=self.node,
            action_type=test_actions.Fibonacci,
            action_name="fibonacci",
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),  # needed?
            execute_callback=self.execute_goal_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            result_timeout=10
        )

    def goal_callback(self, goal_request):
        """
        Args:
            goal_request: of <action_type>.GoalRequest with members
                goal_id (unique_identifier.msgs.UUID) and those specified in the action
        """
        self.node.get_logger().info("received a goal")
        return rclpy.action.server.GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.node.get_logger().info("handle accepted")
        with self.goal_lock:
            self.goal_handle = goal_handle
            goal_handle.execute()

    async def execute_goal_callback(
            self,
            goal_handle: rclpy.action.server.ServerGoalHandle
         ):
        """
        Check for pre-emption, but otherwise just spin around gradually incrementing
        a hypothetical 'percent' done.

        Args:
            goal_handle (:class:`~rclpy.action.server.ServerGoalHandle`): the goal handle of the executing action
        """
        # goal.details (e.g. pose) = don't care
        self.node.get_logger().info("executing a goal")

        frequency = 3.0  # hz
        duration = 5.0
        increment = 100 / (frequency * duration)
        percent_completed = 0.0
        feedback_message = test_actions.Fibonacci.Feedback()
        while True:
            if goal_handle.is_active:
                # if percent_completed >= 50.0:
                #     print(self.foo)
                if percent_completed >= 100.0:
                    percent_completed = 100.0
                    self.node.get_logger().info("feedback 100%%")
                    result = test_actions.Fibonacci.Result()
                    result.sequence = feedback_message.sequence
                    self.node.get_logger().info(str(result.sequence))
                    goal_handle.set_succeeded()
                    return result
                else:
                    self.node.get_logger().info(
                        "feedback {percent:.2f}%%".format(
                            percent=percent_completed
                        )
                    )
                    percent_completed += increment
                    feedback_message.sequence.append(int(percent_completed))
                    goal_handle.publish_feedback(feedback_message)
                    time.sleep(1.0 / frequency)
            else:  # ! active
                self.node.get_logger().info("aborted")
                return test_actions.Fibonacci.Result()

    def abort(self):
        with self.goal_lock:
            if self.goal_handle and self.goal_handle.is_active:
                self.node.get_logger().info("aborting...")
                self.goal_handle.set_aborted()

    def shutdown(self):
        print("middle of shutdown")
        self.action_server.destroy()
        self.node.destroy_node()


if __name__ == '__main__':
    rclpy.init()  # picks up sys.argv automagically internally
    fibonacci = GenericServer()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(fibonacci.node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        fibonacci.abort()
        executor.spin_once(timeout_sec=0.5)
        print("Keyboard Interrupt")

    fibonacci.shutdown()
    executor.shutdown()
    rclpy.shutdown()
