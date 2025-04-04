#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: ChanHyeong Lee

import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String


class HomeServiceChallengeCore(Node):

    def __init__(self):
        super().__init__(
            'home_service_challenge_core',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.target_marker_pub = self.create_publisher(
            Int32,
            '/target_marker_id',
            10
        )
        self.manipulator_pub = self.create_publisher(
            String,
            '/manipulator_control',
            10
        )
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.scenario_sub = self.create_subscription(
            String,
            '/scenario_selection',
            self.scenario_callback,
            10
        )
        self.is_parking_completed_sub = self.create_subscription(
            Bool,
            '/is_parking_completed',
            self.parking_callback,
            10
        )
        self.is_manipulator_completed_sub = self.create_subscription(
            Bool,
            '/is_manipulator_completed',
            self.manipulator_callback,
            10
        )

        self.selected_scenario = None
        self.scenario_running = False
        self.parking_event = threading.Event()
        self.manipulator_event = threading.Event()

    def scenario_callback(self, msg: String):
        if not self.scenario_running:
            self.selected_scenario = msg.data
            self.get_logger().info(f'Selected scenario: {self.selected_scenario}')
            thread = threading.Thread(target=self.run_scenario)
            thread.start()
        else:
            self.get_logger().info('A scenario is already running.')

    def parking_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Parking completed')
            self.parking_event.set()

    def manipulator_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Manipulator task completed.')
            self.manipulator_event.set()

    def list_to_pose_stamped(self, pose_list: list) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pose_list[0]
        pose_msg.pose.position.y = pose_list[1]
        pose_msg.pose.position.z = pose_list[2]
        pose_msg.pose.orientation.x = pose_list[3]
        pose_msg.pose.orientation.y = pose_list[4]
        pose_msg.pose.orientation.z = pose_list[5]
        pose_msg.pose.orientation.w = pose_list[6]
        return pose_msg

    def run_scenario(self):
        self.scenario_running = True

        target_marker_id = 'scenario.' + self.selected_scenario + '.target_marker_id'
        goal_marker_id = 'scenario.' + self.selected_scenario + '.goal_marker_id'
        goal_pose_list = 'scenario.' + self.selected_scenario + '.goal_pose'
        end_pose_list = 'scenario.' + self.selected_scenario + '.end_pose'

        target_marker_id = self.get_parameter(
            target_marker_id).get_parameter_value().integer_value
        goal_marker_id = self.get_parameter(
            goal_marker_id).get_parameter_value().integer_value
        goal_pose_list = self.get_parameter(
            goal_pose_list).get_parameter_value().double_array_value
        end_pose_list = self.get_parameter(
            end_pose_list).get_parameter_value().double_array_value

        marker_msg = Int32()
        marker_msg.data = target_marker_id
        self.target_marker_pub.publish(marker_msg)

        self.parking_event.clear()
        if not self.parking_event.wait(timeout=90.0):
            self.get_logger().error('Parking did not complete within timeout.')
            self.scenario_running = False
            return

        pick_msg = String()
        pick_msg.data = 'pick_target'
        self.manipulator_pub.publish(pick_msg)

        self.manipulator_event.clear()
        if not self.manipulator_event.wait(timeout=30.0):
            self.get_logger().error('Manipulator pick_target did not complete within timeout.')
            self.scenario_running = False
            return
        self.get_logger().info('Manipulator pick_target completed.')

        self.get_logger().info('Sending navigation goal to target location.')
        nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.list_to_pose_stamped(goal_pose_list)

        if not nav_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation action server not available!')
            self.scenario_running = False
            return

        send_goal_future = nav_action_client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            self.scenario_running = False
            return

        self.get_logger().info('Navigation goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.1)

        result = result_future.result()
        if result.status != 4:
            self.get_logger().error('Navigation failed!')
            self.scenario_running = False
            return

        self.get_logger().info('Navigation reached target location.')

        goal_marker_msg = Int32()
        goal_marker_msg.data = goal_marker_id
        self.target_marker_pub.publish(goal_marker_msg)

        self.parking_event.clear()
        if not self.parking_event.wait(timeout=90.0):
            self.get_logger().error('Parking did not complete within timeout.')
            self.scenario_running = False
            return

        place_msg = String()
        place_msg.data = 'place_target'
        self.manipulator_pub.publish(place_msg)

        self.manipulator_event.clear()
        if not self.manipulator_event.wait(timeout=30.0):
            self.get_logger().error('Manipulator place_target did not complete within timeout.')
            self.scenario_running = False
            return
        self.get_logger().info('Manipulator place_target completed.')

        self.get_logger().info('Publishing nav2 message to home.')
        end_pose_msg = self.list_to_pose_stamped(end_pose_list)
        self.nav_goal_pub.publish(end_pose_msg)

        self.get_logger().info('Scenario completed.')
        self.scenario_running = False


def main(args=None):
    rclpy.init(args=args)
    node = HomeServiceChallengeCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
