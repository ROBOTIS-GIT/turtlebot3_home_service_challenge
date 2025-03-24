/*******************************************************************************
<<<<<<< Updated upstream
 * Copyright 2024 ROBOTIS CO., LTD.
=======
 * Copyright 2025 ROBOTIS CO., LTD.
>>>>>>> Stashed changes
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

/* Authors: Wonho Yoon, Sungho Woo, ChanHyeong Lee */

#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class ManipulatorController : public rclcpp::Node
{
public:
  ManipulatorController()
  : Node("manipulator_controller")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/manipulator_control",
      10,
      std::bind(&ManipulatorController::controlCallback, this, std::placeholders::_1)
    );

    manipulator_completed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/is_manipulator_completed", 10);
  }

<<<<<<< Updated upstream
  void initializeMoveGroups(const std::shared_ptr<rclcpp::Node> &node_ptr)
=======
  void initializeMoveGroups(const std::shared_ptr< rclcpp::Node > & node_ptr)
>>>>>>> Stashed changes
  {
    arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr,
      "arm");
    gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node_ptr,
      "gripper");
  }

private:
  void controlCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string command = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

    if (command == "pick_target") {
      pickTarget();
    } else if (command == "place_target") {
      placeTarget();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command received: %s", command.c_str());
    }
  }

  void pickTarget()
  {
    gripper_move_group_->setNamedTarget("open");
    if (gripper_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Gripper opened successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
    }

    arm_move_group_->setNamedTarget("target");
    if (arm_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Arm moved to target (pick_target) successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move arm to target (pick_target)");
    }

    gripper_move_group_->setNamedTarget("close");
    if (gripper_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Gripper closed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to close gripper");
    }

    arm_move_group_->setNamedTarget("home");
    if (arm_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Arm moved to init_pose successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move arm to init_pose");
    }
    std::this_thread::sleep_for(2s);

    publishManipulatorCompleted();
  }

  void placeTarget()
  {
    arm_move_group_->setNamedTarget("target");
    if (arm_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Arm moved to place_target successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move arm to place_target");
    }

    gripper_move_group_->setNamedTarget("open");
    if (gripper_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Gripper opened successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open gripper");
    }

    arm_move_group_->setNamedTarget("home");
    if (arm_move_group_->move()) {
      RCLCPP_INFO(this->get_logger(), "Arm moved to init_pose successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to move arm to init_pose");
    }
    std::this_thread::sleep_for(2s);

    publishManipulatorCompleted();
  }

  void publishManipulatorCompleted()
  {
    auto completed_msg = std_msgs::msg::Bool();
    completed_msg.data = true;
    manipulator_completed_pub_->publish(completed_msg);
    RCLCPP_INFO(this->get_logger(), "Published /is_manipulator_completed: true");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manipulator_completed_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManipulatorController>();
  node->initializeMoveGroups(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
