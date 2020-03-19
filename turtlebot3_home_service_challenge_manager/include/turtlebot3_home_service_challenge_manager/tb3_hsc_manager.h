/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
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

/* Author: Kayman Jung */

#ifndef TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_TASK_MANAGER_H
#define TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_TASK_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalID.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

#include <yaml-cpp/yaml.h>

#include <boost/thread.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include "service.h"

namespace turtlebot3_home_service_challenge
{

namespace TASK {
enum STATUS
{
  SUCCESS = 0,
  FAIL = 1,
  TIME_OUT = 2,
  PAUSE = 3,
  STOP = 4,
};

}

class TaskManager
{
public:
  // enum
  enum COMMAND
  {
    NONE = 0,
    READY = 1,
    STOP = 2,
    PAUSE = 3,
    RESUME = 4,
    COUNT
  };

  enum OBSTACLE
  {
    SAFE = 0,
    WARNING = 1,
    DANGER = 2,
  };

  // const

  // constructor
  TaskManager();

  // method
  // mission
  void start_mission();
  void pause_mission();
  void resume_mission();
  void restart_mission(const std::string &mission_name);
  void stop_mission();

  // task
  void ready_task();
  void finish_task();
  void control_task(COMMAND command);

  // variable
  bool DEBUG;

protected:

  // enum

  // const
  const int SLEEP_MS = 100;

  // method
  void callback_thread();

  // common
  void get_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion);
  void get_euler_angle(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
  bool get_target_pose(const std::string& target_name, geometry_msgs::Pose& target_pose);
  void load_task_data(const std::string& path);
  void load_config(const std::string& path);
  bool is_possible_to_find(const std::string &target_name);

  // mission
  void mission_thread(const std::string &start_mission);
  void on_start_mission();
  void on_finish_mission();

  // task
  void ready_task_thread();
  void finish_task_thread();

  void run_scenario(const std::string &start_scenario);
  void scenario_thread(const std::string &start_scenario);
  TASK::STATUS handle_scenario(const YAML::Node &current_scenario);

  // mobile
  TASK::STATUS approach(const std::string &target_name, int repeat_number);
  bool approach_target(const std::string &target_name);
  bool approach_target(const std::string &target_name, int total_count, int present_count);
  void approach_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose, bool is_final_approach);
  TASK::STATUS leave(double distance);
  void leave_target(const std::string &command);
  void leave_target(double distance);
  void leave_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose);
  TASK::STATUS turn(const geometry_msgs::Pose &target_pose);
  void turn_to_target(const std::string &target_name);
  void turn_to_target(const geometry_msgs::Pose &target_pose);
  void turn_to_target_thread(double yaw);
  void cancel_nav();
  TASK::STATUS navigation(const std::string &target_name, bool recovery, int time_out);
  TASK::STATUS navigation(const std::string &target_name, const std::string& real_target, bool recovery, int time_out);
  TASK::STATUS navigation(const geometry_msgs::Pose &target_pose, bool recovery, int time_out);
  TASK::STATUS navigation(const geometry_msgs::Pose &target_pose, const std::string& real_target, bool recovery, int time_out);
  bool nav_to_target(const std::string& target_name);
  bool nav_to_target(const geometry_msgs::Pose &target_pose);
  bool nav_to_target(const std::string& target_name, const std::string& real_target);
  bool nav_to_target(const geometry_msgs::Pose &target_pose, const std::string& real_target);
  void nav_to_target_thread(const geometry_msgs::Pose &target_pose, const std::string &real_target);
  TASK::STATUS find_target(const std::string &target_name);
  void look_around(const std::string& target_name);
  void look_around_thread(int direction, const std::string& target_name);
  bool check_distance(const std::string& from, const std::string& to, double max_distance);
  bool check_distance(const std::string& from, const geometry_msgs::Pose& to, double max_distance);


  // manipulation
  void init_manipulation();
  TASK::STATUS manipulation_joint(const std::string &target_pose);
  TASK::STATUS manipulation_task(const std::string &target_pose);
  TASK::STATUS manipulation_task(const geometry_msgs::Point& target_position);
  void move_arm_joint(const std::string& target_pose);
  void move_arm_task(const std::string& target_pose);
  void move_arm_task(const geometry_msgs::Point& target_position);
  void move_arm_joint_space_thread(const std::vector<std::string>& joint_name, const std::vector<double>& joint_angle, double path_time);
  void move_arm_task_space_thread(const geometry_msgs::Point& kinematics_position, double path_time);
  TASK::STATUS gripper(bool open);
  void open_gripper();
  void close_gripper();
  void move_gripper_thread(double gripper_position);

  // thread
  void on_stop_mission();
  void on_stop_task();
  bool sleep_for(int sleep_interval, int after_interval, bool &running_condition, bool &pause_condition, bool &termination_condition);
  bool sleep_for(int sleep_interval, int after_interval, bool &running_condition, bool &pause_condition, bool &termination_condition, int time_out);

  // publish
  void publish_goal_nav_msg(const geometry_msgs::PoseStamped& goal_msg);
  void publish_cmd_vel_msg(const geometry_msgs::Twist& msg);
  void publish_approach_marker(bool clear, const std::vector<geometry_msgs::Pose2D> &pose_list);
  void publish_reset_turtlebot();
  void publish_init_pose(const geometry_msgs::PoseWithCovariance& msg);

  // callback
  void command_msg_callback(const std_msgs::String::ConstPtr& msg);
  void navigation_result_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);



  // variable
  std::string robot_name_;
  bool is_running_mission_;
  bool is_running_task_thread_;
  bool is_running_sub_task_thread_;
  bool is_ready_mission_;
  bool is_stop_mission_;
  bool is_pause_mission_;
  bool is_stop_;
  bool is_pause_;
  bool mission_result_;
  bool task_result_;
  boost::thread* moving_thread_;
  boost::thread* task_thread_;
  std::string navigation_goal_id_;
  int navigation_status_;
  int approach_interval_sleep_ms_;
  int repeat_times_;
  double approach_linear_vel_;
  double approach_angular_vel_;
  double approach_distance_;
  int leave_interval_sleep_ms_;
  double leave_linear_vel_;
  double leave_angular_vel_;
  int obstacle_status_;

  std::vector<geometry_msgs::Pose2D> approach_pose_list_;
  std::vector<std::string> marker_name_list_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  std::map<std::string, Service*> room_service_list_;
  std::map<std::string, std::map<std::string, double>> arm_pose_list_;

  //scenario
  geometry_msgs::Pose last_pose_;

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::MoveGroupInterface* move_group2_;

  // publisher/subscriber
  ros::Publisher goal_nav_pub_;
  ros::Publisher cancel_nav_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher debug_marker_pub_;
  ros::Publisher reset_turtlebot_pub_;
  ros::Publisher init_pose_pub_;
  ros::Publisher gripper_moving_time_pub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber navigation_result_sub_;
  ros::Subscriber laser_scan_sub_;

};
}
#endif // TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_TASK_MANAGER_H
