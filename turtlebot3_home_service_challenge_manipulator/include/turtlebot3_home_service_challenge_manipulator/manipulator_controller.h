/*******************************************************************************
 * Copyright 2025 ROBOTIS CO., LTD.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

/* Authors: ChanHyeong Lee */

#ifndef TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANIPULATOR__MANIPULATOR_CONTROLLER_H_
#define TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANIPULATOR__MANIPULATOR_CONTROLLER_H_

#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

class ManipulatorController: public rclcpp::Node
{
public:
  ManipulatorController();

  void initializeMoveGroups(const std::shared_ptr <rclcpp::Node> & node_ptr);

private:
  void controlCallback(const std_msgs::msg::String::SharedPtr msg);

  void pickTarget();
  void placeTarget();
  void publishManipulatorCompleted();

  rclcpp::Subscription<std_msgs::msg::String> ::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool> ::SharedPtr manipulator_completed_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
};

#endif  // TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANIPULATOR__MANIPULATOR_CONTROLLER_H_
