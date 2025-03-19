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

import rclpy
from rclpy.node import Node
import math

import tf2_ros
import tf_transformations

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Bool


class ArUcoParking(Node):
    def __init__(self):
        super().__init__('aruco_parking')
        self.create_subscription(Int32, '/target_marker_id', self.marker_callback, 10)
        self.parking_pub = self.create_publisher(Bool, '/is_parking_completed', 10)

        self.target_marker_id = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(1.0, self.send_goal)

    def marker_callback(self, msg: Int32):
        self.target_marker_id = msg.data
        self.get_logger().info(f'Received target marker id: {self.target_marker_id}')

    def send_goal(self):
        if self.target_marker_id is None:
            return

        marker_frame = f'ar_marker_{self.target_marker_id}'

        try:
            trans = self.tf_buffer.lookup_transform('map', marker_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().info(f'Transform from map to {marker_frame} not available yet: {e}')
            return

        marker_x = trans.transform.translation.x
        marker_y = trans.transform.translation.y
        marker_z = trans.transform.translation.z

        q = trans.transform.rotation
        q_tuple = [q.x, q.y, q.z, q.w]
        marker_z_vector = tf_transformations.quaternion_matrix(q_tuple)[:3, 2]

        offset = 0.23
        goal_x = marker_x + offset * marker_z_vector[0]
        goal_y = marker_y + offset * marker_z_vector[1]
        goal_z = marker_z + offset * marker_z_vector[2]

        vec = [-marker_z_vector[0], -marker_z_vector[1]]
        yaw = math.atan2(vec[1], vec[0])
        self.get_logger().info(
            f'Goal position: ({goal_x:.2f}, {goal_y:.2f}, {goal_z:.2f}), yaw: {yaw:.2f}')

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = goal_z
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        goal_msg.pose = pose

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.target_marker_id = None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        parking_msg = Bool()
        parking_msg.data = True
        self.parking_pub.publish(parking_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoParking()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
