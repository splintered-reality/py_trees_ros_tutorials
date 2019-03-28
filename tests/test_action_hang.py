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

import time
import uuid

import functools

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from test_msgs.action import Fibonacci

from unique_identifier_msgs.msg import UUID


class MockActionClient():

    def __init__(self, node):
        self.reset()
        self.goal_srv = node.create_client(
            Fibonacci.GoalRequestService, '/fibonacci/_action/send_goal')
        self.cancel_srv = node.create_client(
            Fibonacci.CancelGoalService, '/fibonacci/_action/cancel_goal')
        self.result_srv = node.create_client(
            Fibonacci.GoalResultService, '/fibonacci/_action/get_result')
        self.feedback_sub = node.create_subscription(
            Fibonacci.Feedback, '/fibonacci/_action/feedback', self.feedback_callback)
        self.status_sub = node.create_subscription(
            Fibonacci.GoalStatusMessage, '/fibonacci/_action/status', self.status_callback)

    def reset(self):
        self.feedback_msg = None
        self.status_msg = None

    def feedback_callback(self, feedback_msg):
        self.feedback_msg = feedback_msg

    def status_callback(self, status_msg):
        self.status_msg = status_msg

    def send_goal(self, goal_msg):
        return self.goal_srv.call_async(goal_msg)

    def cancel_goal(self, cancel_msg):
        return self.cancel_srv.call_async(cancel_msg)

    def get_result(self, goal_uuid):
        result_request = Fibonacci.GoalResultService.Request()
        result_request.action_goal_id = goal_uuid
        return self.result_srv.call_async(result_request)


def execute_long_goal_callback(goal_handle, node):
    increment = 10
    percent_completed = 0.0
    while rclpy.ok:
        if goal_handle.is_active:
            if percent_completed >= 100.0:
                percent_completed = 100.0
                node.get_logger().info("feedback 100%%")
                result = Fibonacci.Result()
                goal_handle.set_succeeded()
                return result
            else:
                node.get_logger().info(
                    "feedback {percent:.2f}%%".format(percent=percent_completed)
                )
                percent_completed += increment
        time.sleep(0.3)
    return Fibonacci.Result()


if __name__ == '__main__':
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('test_action_server', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    mock_action_client = MockActionClient(node)
    action_server = ActionServer(
        node,
        Fibonacci,
        'fibonacci',
        execute_callback=functools.partial(execute_long_goal_callback, node=node)
    )

    goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
    goal_msg = Fibonacci.Goal()
    goal_msg.action_goal_id = goal_uuid
    goal_future = mock_action_client.send_goal(goal_msg)
    print("DJS")
    # rclpy.spin_until_future_complete(node, goal_future, executor)
    print("DJS2")
    # goal_handle = goal_future.result()

    node.get_logger().info("get result future")
    get_result_future = mock_action_client.get_result(goal_uuid)
    while context.ok() and not get_result_future.done():
        node.get_logger().info("spinning")
        executor.spin_once(timeout_sec=0.1)
    # rclpy.spin_until_future_complete(node, get_result_future, executor)
    result = get_result_future.result()

    node.get_logger().info("action server destroy")
    action_server.destroy()
    node.get_logger().info("remove node")
    executor.remove_node(node)
    print("destroy node")
    node.destroy_node()
    executor.shutdown()
    rclpy.shutdown(context=context)
