#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time

from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('minimal_action_server')
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info("handle accepted")
        with self._goal_lock:
            self._goal_handle = goal_handle
            goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Executes a goal."""
        self.get_logger().info('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Start executing the action
        for i in range(1, goal_handle.request.order):

            with self._goal_lock:

                if goal_handle.is_cancel_requested:
                    goal_handle.set_canceled()
                    self.get_logger().info('Goal canceled')
                    return Fibonacci.Result()

                if not goal_handle.is_active:
                    self.get_logger().info("aborted")
                    return Fibonacci.Result()

                # Update Fibonacci sequence
                feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

                self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

                # Publish the feedback
                goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.set_succeeded()

        # Populate result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result

    def abort(self):
        with self._goal_lock:
            if self._goal_handle and self._goal_handle.is_active:
                self.get_logger().info("aborting...")
                self._goal_handle.set_aborted()


def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(minimal_action_server, executor=executor)
    except KeyboardInterrupt:
        minimal_action_server.abort()
        executor.spin_once(timeout_sec=0.5)
        executor.spin_once(timeout_sec=0.5)
        executor.spin_once(timeout_sec=0.5)
        executor.shutdown()

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
