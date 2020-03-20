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

#include "turtlebot3_home_service_challenge_manager/tb3_hsc_manager.h"

namespace turtlebot3_home_service_challenge
{
TaskManager::TaskManager()
  : DEBUG(false),
    mission_result_(true),
    task_result_(true),
    is_running_mission_(false),
    is_running_task_thread_(false),
    is_running_sub_task_thread_(false),
    is_stop_mission_(false),
    is_pause_mission_(false),
    is_stop_(false),
    is_pause_(false),
    repeat_times_(3),
    approach_interval_sleep_ms_(500),
    approach_linear_vel_(0.05),
    approach_angular_vel_(0.1),
    approach_distance_(0.1),
    leave_interval_sleep_ms_(500),
    leave_linear_vel_(0.05),
    leave_angular_vel_(0.1),
    navigation_status_(actionlib_msgs::GoalStatus::PENDING),
    obstacle_status_(SAFE)
{
  robot_name_ = "/tb3_hsc";

  marker_name_list_.push_back("ar_marker_0");
  marker_name_list_.push_back("ar_marker_1");
  marker_name_list_.push_back("ar_marker_2");
  marker_name_list_.push_back("ar_marker_3");
  marker_name_list_.push_back("ar_marker_4");
  marker_name_list_.push_back("ar_marker_5");
  marker_name_list_.push_back("ar_marker_6");
  marker_name_list_.push_back("ar_marker_7");

  boost::thread queue_thread = boost::thread(boost::bind(&TaskManager::callback_thread, this));

  ros::NodeHandle p_nh("~");
  std::string task_data_path = p_nh.param<std::string>("task_data_path", ros::package::getPath(ROS_PACKAGE_NAME) + "/config/room.yaml");
  load_task_data(task_data_path);
  std::string config_data_path = p_nh.param<std::string>("config_data_path", ros::package::getPath(ROS_PACKAGE_NAME) + "/config/config.yaml");
  load_config(config_data_path);

  init_manipulation();
}

void TaskManager::callback_thread()
{
  // subscriber & publisher
  ros::NodeHandle nh;

  goal_nav_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_name_ + "/move_base_simple/goal", 0);
  cancel_nav_pub_ = nh.advertise<actionlib_msgs::GoalID>(robot_name_ + "/move_base/cancel", 0);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(robot_name_ + "/cmd_vel", 0);
  debug_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(robot_name_ + "/marker", 0);
  reset_turtlebot_pub_ = nh.advertise<std_msgs::Empty>(robot_name_ + "/reset", 0);
  init_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(robot_name_ + "/initialpose", 0);
  gripper_moving_time_pub_ = nh.advertise<std_msgs::Float64>(robot_name_ + "/gripper_move_time", 0);

  cmd_sub_ = nh.subscribe(robot_name_ + "/command", 1, &TaskManager::command_msg_callback, this);
  navigation_result_sub_ = nh.subscribe(robot_name_ + "/move_base/status", 1, &TaskManager::navigation_result_callback, this);
  laser_scan_sub_ = nh.subscribe(robot_name_ + "/scan", 1, &TaskManager::laser_scan_callback, this);

  tf_listener_.reset( new tf::TransformListener());

  ros::Duration dur(0.01);

  while (nh.ok())
  {
    ros::spinOnce();

    dur.sleep();
  }
}


// ========================================
// ==========      Common
// ========================================
void TaskManager::get_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  quaternion.w = cy * cp * cr + sy * sp * sr;
  quaternion.x = cy * cp * sr - sy * sp * cr;
  quaternion.y = sy * cp * sr + cy * sp * cr;
  quaternion.z = sy * cp * cr - cy * sp * sr;
}

void TaskManager::get_euler_angle(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
  double cosr_cosp = +1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  double cosy_cosp = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  yaw = atan2(siny_cosp, cosy_cosp);
}

bool TaskManager::get_target_pose(const std::string& target_name, geometry_msgs::Pose& target_pose)
{
  tf::StampedTransform desired_transform;

  std::string base_name = "/map";
  try
  {
    tf_listener_->lookupTransform(base_name, target_name, ros::Time(0), desired_transform);
    Eigen::Vector3d transform_position(desired_transform.getOrigin().x(),
                                       desired_transform.getOrigin().y(),
                                       desired_transform.getOrigin().z());
    Eigen::Quaterniond transform_orientation(desired_transform.getRotation().w(),
                                             desired_transform.getRotation().x(),
                                             desired_transform.getRotation().y(),
                                             desired_transform.getRotation().z());

    tf::pointEigenToMsg(transform_position, target_pose.position);
    tf::quaternionEigenToMsg(transform_orientation, target_pose.orientation);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_COND(DEBUG, "%s",ex.what());
    return false;
  }

  return true;
}

void TaskManager::load_task_data(const std::string& path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

    room_service_list_.clear();

    for (YAML::iterator yaml_it = doc.begin(); yaml_it != doc.end(); ++yaml_it)
    {
      std::string task_name;

      task_name = yaml_it->first.as<std::string>();

      YAML::Node sub_node = yaml_it->second;
      std::string name = sub_node["name"].as<std::string>();

      YAML::Node object_node = sub_node["object"];
      std::string object_name = object_node["marker"].as<std::string>();
      std::vector<double> object_position = object_node["position"].as< std::vector<double> >();

      YAML::Node target_node = sub_node["target"];
      std::string target_name = target_node["marker"].as<std::string>();
      std::vector<double> target_position = target_node["position"].as< std::vector<double> >();

      std::vector<double> room_x = sub_node["x"].as< std::vector<double> >();
      std::vector<double> room_y = sub_node["y"].as< std::vector<double> >();

      if(room_x.size() != 2 || room_y.size() != 2)
        return;

      Service *service = new Service(task_name);
      service->set_object(object_name);
      service->set_object_position(object_position);
      service->set_target(target_name);
      service->set_target_position(target_position);
      service->set_room_x(room_x.at(0), room_x.at(1));
      service->set_room_y(room_y.at(0), room_y.at(1));

      room_service_list_[task_name] = service;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_COND(DEBUG, "Fail to load task data.");
    room_service_list_.clear();
    return;
  }
}

void TaskManager::load_config(const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

    // arm pose
    YAML::Node arm_pose_node = doc["arm_pose"];
    arm_pose_list_.clear();

    for (YAML::iterator yaml_it = arm_pose_node.begin(); yaml_it != arm_pose_node.end(); ++yaml_it)
    {
      std::string pose_name;

      pose_name = yaml_it->first.as<std::string>();

      YAML::Node sub_node = yaml_it->second;

      std::map<std::string, double> pose;

      for (YAML::iterator sub_it = sub_node.begin(); sub_it != sub_node.end(); ++sub_it)
      {
        std::string joint_name = sub_it->first.as<std::string>();
        double joint_angle = sub_it->second.as<double>() * M_PI / 180.0;

        pose[joint_name] = joint_angle;
      }

      arm_pose_list_[pose_name] = pose;
    }

    // approach
    YAML::Node approach_node = doc["approach"];
    approach_distance_ = approach_node["distance"].as<double>();
    approach_linear_vel_ = approach_node["linear_vel"].as<double>();
    approach_angular_vel_ = approach_node["angular_vel"].as<double>();
    approach_interval_sleep_ms_ = approach_node["interval_sleep_ms"].as<int>();
    repeat_times_ = approach_node["repeat_times"].as<int>();
    if(repeat_times_ < 1)
      repeat_times_ = 1;

    // leave
    YAML::Node leave_node = doc["leave"];
    leave_linear_vel_ = leave_node["linear_vel"].as<double>();
    leave_angular_vel_ = leave_node["angular_vel"].as<double>();
    leave_interval_sleep_ms_ = leave_node["interval_sleep_ms"].as<int>();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM_COND(DEBUG, "Fail to load config data." << e.what());
    arm_pose_list_.clear();
    return;
  }
}


// ========================================
// ==========      ROS
// ========================================
void TaskManager::publish_goal_nav_msg(const geometry_msgs::PoseStamped& goal_msg)
{
  ROS_INFO_STREAM_COND(DEBUG, "Publish Nav msg");
  goal_nav_pub_.publish(goal_msg);
}

void TaskManager::publish_cmd_vel_msg(const geometry_msgs::Twist& msg)
{
  ROS_INFO_STREAM_COND(DEBUG, "Publish Cmd_vel msg : " << msg.linear.x << ", " << msg.angular.z);
  cmd_vel_pub_.publish(msg);
}

void TaskManager::publish_approach_marker(bool clear, const std::vector<geometry_msgs::Pose2D>& pose_list)
{
  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  visualization_msgs::Marker arrow_marker;
  visualization_msgs::Marker cube_marker;

  arrow_marker.header.frame_id = "map";
  arrow_marker.header.stamp = now;
  arrow_marker.ns = "approach_marker";
  cube_marker = arrow_marker;

  arrow_marker.id = 0;
  cube_marker.id = 100;

  arrow_marker.type = visualization_msgs::Marker::ARROW;
  cube_marker.type = visualization_msgs::Marker::CUBE;
  arrow_marker.action = (clear == false) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETEALL;
  cube_marker.action = arrow_marker.action;

  if(clear == true)
  {
    marker_array.markers.push_back(arrow_marker);
    debug_marker_pub_.publish(marker_array);
    return;
  }

  arrow_marker.scale.x = 0.15;
  arrow_marker.scale.y = 0.015;
  arrow_marker.scale.z = 0.015;

  arrow_marker.color.r = 0.0;
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 1.0;

  cube_marker.scale.x = 0.03;
  cube_marker.scale.y = 0.03;
  cube_marker.scale.z = 0.03;

  cube_marker.color.r = 0.0;
  cube_marker.color.g = 1.0;
  cube_marker.color.b = 1.0;
  cube_marker.color.a = 1.0;

  for(auto it = pose_list.begin(); it != pose_list.end(); ++it)
  {
    arrow_marker.pose.position.x = it->x;
    arrow_marker.pose.position.y = it->y;
    arrow_marker.pose.position.z = 0.15;

    get_quaternion(0.0, 0.0, it->theta, arrow_marker.pose.orientation);
    marker_array.markers.push_back(arrow_marker);
    cube_marker.pose = arrow_marker.pose;
    marker_array.markers.push_back(cube_marker);

    arrow_marker.id++;
    cube_marker.id++;
  }

  debug_marker_pub_.publish(marker_array);
}

void TaskManager::publish_reset_turtlebot()
{
  std_msgs::Empty msg;

  reset_turtlebot_pub_.publish(msg);
  ROS_INFO("RESET Turtlebot!!");
}

void TaskManager::publish_init_pose(const geometry_msgs::PoseWithCovariance& msg)
{
  geometry_msgs::PoseWithCovarianceStamped init_msg;
  init_msg.header.frame_id = "map";
  init_msg.header.stamp = ros::Time::now();
  init_msg.pose = msg;

  init_pose_pub_.publish(init_msg);
}

void TaskManager::command_msg_callback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data.find("mission") != std::string::npos)
  {
    if(msg->data == "start_mission")
      start_mission();
    else if(msg->data == "ready_mission")
      ready_task();
    else if(msg->data == "finish_mission")
      finish_task();
    else if(msg->data == "stop_mission")
      stop_mission();
    else if(msg->data == "pause_mission")
      pause_mission();
    else if(msg->data == "resume_mission")
      resume_mission();
    else if(msg->data.find("restart_mission") != std::string::npos)
    {
      std::size_t pos = msg->data.find(":");
      if(pos != std::string::npos)
      {
        std::string mission_name = msg->data.substr(pos+1);
        restart_mission(mission_name);
      }
    }

    return;
  }

  // check command for controlling the task
  if(msg->data.find("task") != std::string::npos)
  {
    COMMAND current_command = NONE;
    if(msg->data == "stop_task")
      current_command = STOP;
    else if(msg->data == "pause_task")
      current_command = PAUSE;
    else if(msg->data == "resume_task")
      current_command = RESUME;
    else if(msg->data == "ready_task")
      current_command = READY;

    if(current_command != NONE)
    {
      control_task(current_command);
      return;
    }
  }

  // command for testing
  if(is_running_sub_task_thread_ == true)
  {
    ROS_WARN("Moving thread is running, command is ignored.");

    return;
  }
  else
  {
    // navigation
    if(msg->data.find("nav") != std::string::npos)
      nav_to_target(msg->data);

    // approach
    else if(msg->data.find("approach") != std::string::npos)
      approach_target(msg->data);

    else if(msg->data.find("leave") != std::string::npos)
      leave_target(msg->data);

    else if(msg->data.find("find") != std::string::npos)
      look_around(msg->data);

    else if(msg->data == "arm_home")
      move_arm_joint("home_with_object");

    else if(msg->data == "arm_joint")
      move_arm_joint("via_pose");

    else if(msg->data == "arm_task")
    {
      geometry_msgs::Point target;
      target.x = 0.24;
      target.y = 0.0;
      target.z = 0.15;

      move_arm_task(target);
    }

    else if(msg->data == "arm_task2")
    {
      geometry_msgs::Point target;
      target.x = 0.24;
      target.y = 0.0;
      target.z = 0.2;

      move_arm_task(target);
    }

    else if(msg->data == "open_gripper")
      open_gripper();

    else if(msg->data == "close_gripper")
      close_gripper();
  }
}

void TaskManager::navigation_result_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if(msg->status_list.size() == 0)
    return;

  // get the status of the last navigatoin goal
  navigation_goal_id_ = msg->status_list.rbegin()->goal_id.id;
  navigation_status_ = msg->status_list.rbegin()->status;
}

// check obstacle(front right/left)
void TaskManager::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double warning_range = 0.18;
  double danger_range = 0.16;

  // check front_left (45 deg ~ 60 deg)
  int start_index = int(((45.0 * M_PI / 180.0) - msg->angle_min) / msg->angle_increment);
  int end_index = int(((60.0 * M_PI / 180.0) - msg->angle_min) / msg->angle_increment);
  double distance_average = 0;

  try {
    double total_distance = 0.0;
    int count = 0;

    for(int ix = start_index; ix <= end_index; ix++)
    {
      if(msg->ranges.at(ix) < msg->range_min || msg->ranges.at(ix) > msg->range_max)
        continue;

      total_distance += msg->ranges.at(ix);
      count++;
    }

    if(count != 0)
    {
      distance_average = total_distance / count;

      if(distance_average < danger_range)
        obstacle_status_ = DANGER;
      else if(distance_average < warning_range)
        obstacle_status_ = WARNING;
      else
        obstacle_status_ = SAFE;
    }
    ROS_INFO_STREAM_COND(DEBUG, "laser scan : [" << start_index << " - " << end_index << "] : " << distance_average << ", " << count);

    // check front_right (300 deg ~ 315 deg)
    start_index = int(((300.0 * M_PI / 180.0) - msg->angle_min) / msg->angle_increment);
    end_index = int(((315.0 * M_PI / 180.0) - msg->angle_min) / msg->angle_increment);
    distance_average = 0;

    total_distance = 0.0;
    count = 0;

    for(int ix = start_index; ix <= end_index; ix++)
    {
      if(msg->ranges.at(ix) < msg->range_min || msg->ranges.at(ix) > msg->range_max)
        continue;

      total_distance += msg->ranges.at(ix);
      count++;
    }
    if(count != 0)
    {
      distance_average = total_distance / count;

      if(distance_average < danger_range)
        obstacle_status_ = DANGER;
      else if(distance_average < warning_range)
        obstacle_status_ = std::max<int>(obstacle_status_, WARNING); //(obstacle_status_ == DANGER) ? DANGER : WARNING;
      else
        obstacle_status_ = std::max<int>(obstacle_status_, SAFE);
    }

  }
  catch (const std::out_of_range* oor) {
    ROS_ERROR_STREAM("Error in checking obstacle : "<< oor->what());
  }

}

}
