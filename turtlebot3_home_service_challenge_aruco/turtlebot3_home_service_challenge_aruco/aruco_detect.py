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

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


class ArUcoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/pi_camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.sub_camera_image = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.ar_size = 0.088
        self.ar_dict_id = 'DICT_5X5_250'

        self.camera_info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        dict_id = getattr(cv2.aruco, self.ar_dict_id)
        self.ar_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.ar_param = cv2.aruco.DetectorParameters_create()
        else:
            self.ar_param = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

    def camera_info_callback(self, camera_info_msg):
        self.camera_info_msg = camera_info_msg
        self.intrinsic_mat = np.reshape(np.array(self.camera_info_msg.k), (3, 3))
        self.distortion = np.array(self.camera_info_msg.d)
        self.destroy_subscription(self.sub_camera_info)

    def image_callback(self, img_msg):
        if self.camera_info_msg is None:
            self.get_logger().warn('No camera info')
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='mono8')

        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.ar_dict, parameters=self.ar_param
        )
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.ar_size, self.intrinsic_mat, self.distortion
            )

            R_corr = Rotation.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()

            for i, marker_id in enumerate(marker_ids):
                tvec = tvecs[i][0]
                rvec = np.array(rvecs[i][0])

                R_marker, _ = cv2.Rodrigues(rvec)
                R_corrected = R_corr @ R_marker
                t_corrected = R_corr @ tvec

                quat_corrected = Rotation.from_matrix(R_corrected).as_quat()

                pose = Pose()
                pose.position.x = t_corrected[0]
                pose.position.y = t_corrected[1]
                pose.position.z = t_corrected[2]
                pose.orientation.x = quat_corrected[0]
                pose.orientation.y = quat_corrected[1]
                pose.orientation.z = quat_corrected[2]
                pose.orientation.w = quat_corrected[3]

                t_msg = TransformStamped()
                t_msg.header.stamp = img_msg.header.stamp
                t_msg.header.frame_id = 'camera_link'
                t_msg.child_frame_id = f'ar_marker_{marker_id[0]}'
                t_msg.transform.translation.x = pose.position.x
                t_msg.transform.translation.y = pose.position.y
                t_msg.transform.translation.z = pose.position.z
                t_msg.transform.rotation.x = pose.orientation.x
                t_msg.transform.rotation.y = pose.orientation.y
                t_msg.transform.rotation.z = pose.orientation.z
                t_msg.transform.rotation.w = pose.orientation.w
                self.br.sendTransform(t_msg)


def main():
    rclpy.init()
    node = ArUcoDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
