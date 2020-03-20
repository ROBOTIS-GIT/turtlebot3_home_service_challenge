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

// ========================================
// ==========     Mobile
// ========================================
TASK::STATUS TaskManager::approach(const std::string &target_name, int repeat_number)
{
  bool continue_result;

  for(int ix = 0; ix < repeat_number; ix++)
  {
    bool approach_result = approach_target(target_name, repeat_number, ix + 1);

    if(approach_result == false)
    {
      // go back and find the target
      // leave
      leave_target("leave_back_inter");

      continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
      if(continue_result == false)
      {
        on_stop_task();
        return TASK::STOP;
      }

      // wait 1 sec
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS * 10));

      // try again
      approach_result = approach_target(target_name, repeat_number, ix + 1);

      if(approach_result == false)
      {
        ROS_ERROR("Failed to approach");
        return TASK::FAIL;
      }
    }

    // wait for approaching
    continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return TASK::STOP;;
    }

    // fail to apprach because of obstacle, try again
    if(task_result_ == false)
    {
      task_result_ = true;
      ix--;
    }

    if(ix == (repeat_number - 1))
      break;

    // leave
    leave_target("leave_back_inter");

    continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return TASK::STOP;;
    }
  }

  return task_result_ ? TASK::SUCCESS : TASK::FAIL;
}

bool TaskManager::approach_target(const std::string &target_name)
{
  return approach_target(target_name, 1, 1);
}

bool TaskManager::approach_target(const std::string& target_name, int total_count, int present_count)
{
  // approach ar_marker
  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      geometry_msgs::Pose target_pose, present_pose;

      std::string base_frame_id = robot_name_ + "/base_footprint";

      bool result = get_target_pose(marker_name, target_pose) &&
          get_target_pose(base_frame_id, present_pose);
      if(result == false)
      {
        ROS_ERROR("Couldn't find the target or present footprint");
        return false;
      }

      double final_offset = approach_distance_ + (total_count - present_count) * 0.05;
      double via_offset = final_offset + 0.05;

      Eigen::Vector3d offset(0, 0, via_offset);

      Eigen::Quaterniond target_orientation;
      Eigen::Vector3d object_position, target_position;
      Eigen::VectorXd global_offset;

      // target position
      tf::quaternionMsgToEigen(target_pose.orientation, target_orientation);
      tf::pointMsgToEigen(target_pose.position, object_position);
      global_offset = target_orientation.toRotationMatrix() * offset;
      global_offset.coeffRef(2) = 0.0;
      target_position = object_position + global_offset;

      tf::pointEigenToMsg(target_position, target_pose.position);

      // target orientation : global yaw
      double target_yaw = atan2(-global_offset.coeff(1), -global_offset.coeff(0));

      geometry_msgs::Pose2D present_pose_2d, target_pose_2d;
      present_pose_2d.x = present_pose.position.x;
      present_pose_2d.y = present_pose.position.y;
      double p_roll, p_pitch, p_yaw;
      get_euler_angle(present_pose.orientation, p_roll, p_pitch, p_yaw);
      present_pose_2d.theta = p_yaw;

      target_pose_2d.x = target_pose.position.x;
      target_pose_2d.y = target_pose.position.y;
      target_pose_2d.theta = target_yaw;

      //publish start and target
      approach_pose_list_.clear();
      approach_pose_list_.push_back(present_pose_2d);
      approach_pose_list_.push_back(target_pose_2d);
      publish_approach_marker(false, approach_pose_list_);

      bool is_final = (total_count == present_count);
      moving_thread_ = new boost::thread(boost::bind(&TaskManager::approach_target_thread, this, present_pose_2d, target_pose_2d, is_final));
      delete moving_thread_;

      return true;
    }
  }
  return false;
}

void TaskManager::approach_target_thread(const geometry_msgs::Pose2D& present_pose, const geometry_msgs::Pose2D& target_pose, bool is_final_approach)
{
  is_running_sub_task_thread_ = true;
  int interval_time = 0;

  ROS_WARN_STREAM_COND(DEBUG, "present : " << present_pose.x << ", " << present_pose.y << " | " << present_pose.theta);
  ROS_WARN_STREAM_COND(DEBUG, "target : " << target_pose.x << ", " << target_pose.y << " | " << target_pose.theta);

  // LINEAR_MAX_VELOCITY = (WHEEL_RADIUS * 2 * M_PI * WAFFLE / 60)       #m/s  (WHEEL_RADIUS = 0.033, BURGER : 61[rpm], WAFFLE : 77[rpm])
  // ANGULAR_MAX_VELOCITY = (MAX_LINEAR_VELOCITY / WAFFLE_TURNING_RADIUS)   #rad/s (WAFFLE_TURNING_RADIUS = 0.1435)

  double diff_x = target_pose.x - present_pose.x;
  double diff_y = target_pose.y - present_pose.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  double yaw_1 = atan2(diff_y, diff_x) - present_pose.theta;
  double yaw_2 = target_pose.theta - atan2(diff_y, diff_x);

  ROS_INFO_STREAM_COND(DEBUG, "yaw_1 : " << (yaw_1 * 180 / M_PI) << ", distance : " << distance << ", yaw_2 : " << (yaw_2 * 180 / M_PI));

  geometry_msgs::Twist approach_msg;
  int moving_time;
  int total_time;

  // turn to yaw_1
  if(yaw_1 != 0.0)
  {
    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = (yaw_1 > 0) ? approach_angular_vel_ : - approach_angular_vel_;
    publish_cmd_vel_msg(approach_msg);

    // wait to turn
    moving_time = yaw_1 * 1000 / approach_msg.angular.z;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + interval_time));
    ROS_WARN_STREAM_COND(DEBUG, "turn 1 : " << (yaw_1 * 180 / M_PI) << ", time(ms) : " << moving_time);

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(approach_interval_sleep_ms_));
  }

  // go to via
  if(distance != 0.0)
  {
    approach_msg.linear.x = approach_linear_vel_;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    // wait to go straight
    moving_time = distance * 1000 / approach_msg.linear.x;
    total_time = moving_time + interval_time;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(total_time % SLEEP_MS));
    for(int ix = 0; (ix * SLEEP_MS) < total_time; ix++)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS));
      if(obstacle_status_ == DANGER)
      {
        ROS_ERROR("Obstacle is close. Stopping aproach!!");
        task_result_ = false;
        break;
      }
    }
    //  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + additional_sleep_ms));
    ROS_WARN_STREAM_COND(DEBUG, "go straight : " << distance << ", time(ms) : " << moving_time);

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(approach_interval_sleep_ms_));
  }

  // turn to target theta
  if(yaw_2 != 0.0)
  {
    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = (yaw_2 > 0) ? approach_angular_vel_ : - approach_angular_vel_;
    publish_cmd_vel_msg(approach_msg);

    // wait to turn
    moving_time = yaw_2 * 1000 / approach_msg.angular.z;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + interval_time));
    ROS_WARN_STREAM_COND(DEBUG, "turn 2 : " << (yaw_2 * 180 / M_PI) << ", time(ms) : " << moving_time);

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);
  }

  boost::this_thread::sleep_for(boost::chrono::milliseconds(approach_interval_sleep_ms_));

  if(is_final_approach)
  {
    double final_approach_distance = 0.05;

    // go to target(final approach)
    approach_msg.linear.x = approach_linear_vel_;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    // wait to go straight
    moving_time = final_approach_distance * 1000 / approach_msg.linear.x;
    total_time = moving_time + interval_time;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(total_time % SLEEP_MS));
    for(int ix = 0; (ix * SLEEP_MS) < total_time; ix++)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS));
      if(obstacle_status_ == DANGER)
      {
        ROS_ERROR("Obstacle is close. Stopping aproach!!");
        task_result_ = false;
        break;
      }
    }

    ROS_WARN_STREAM_COND(DEBUG, "go straight : " << distance << ", time(ms) : " << moving_time);

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);
  }

  is_running_sub_task_thread_ = false;
}

TASK::STATUS TaskManager::leave(double distance)
{
  leave_target(distance);

  bool continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
//        on_stop_task();
    return TASK::STOP;
  }

  return task_result_ ? TASK::SUCCESS : TASK::FAIL;
}

void TaskManager::leave_target(const std::string& command)
{
  double leave_back_range = 0.25;
  double leave_back_inter_range = 0.15;

  if(command.find("back") != std::string::npos)
  {
    publish_approach_marker(true, approach_pose_list_);

    geometry_msgs::Pose2D pose_1, pose_2;

    if(command.find("inter") != std::string::npos)
      pose_2.x = -leave_back_inter_range;
    else
      pose_2.x = -leave_back_range;

    moving_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, pose_1, pose_2));
    delete moving_thread_;
  }
  else
  {
    if(approach_pose_list_.size() != 2)
    {
      ROS_ERROR("No approach data!!!");
      return;
    }

    // clear marker
    publish_approach_marker(true, approach_pose_list_);

    moving_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, approach_pose_list_.at(1), approach_pose_list_.at(0)));
    delete moving_thread_;
  }
}

void TaskManager::leave_target(double distance)
{
  publish_approach_marker(true, approach_pose_list_);

  geometry_msgs::Pose2D pose_1, pose_2;
  pose_2.x = -distance;

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, pose_1, pose_2));
  delete moving_thread_;
}

void TaskManager::leave_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose)
{
  is_running_sub_task_thread_ = true;

  int interval_sleep_ms = 500;

  double diff_x = present_pose.x - target_pose.x;
  double diff_y = present_pose.y - target_pose.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  double yaw_1 = atan2(diff_y, diff_x) - present_pose.theta;
  double yaw_2 = target_pose.theta - atan2(diff_y, diff_x);

  geometry_msgs::Twist approach_msg;
  int moving_time;

  // turn to yaw_1
  if(yaw_1 != 0.0)
  {
    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = (yaw_1 > 0) ? leave_angular_vel_ : - leave_angular_vel_;

    publish_cmd_vel_msg(approach_msg);

    // wait to turn
    moving_time = yaw_1 * 1000 / approach_msg.angular.z;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms));
  }
  // go to target
  if(distance != 0.0)
  {
    approach_msg.linear.x = - leave_linear_vel_;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    // wait to turn
    moving_time = - distance * 1000 / approach_msg.linear.x;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms));
  }

  // turn to target theta
  if(yaw_2 != 0.0)
  {
    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = (yaw_2 > 0) ? leave_angular_vel_ : - leave_angular_vel_;
    publish_cmd_vel_msg(approach_msg);

    // wait to turn
    moving_time = yaw_2 * 1000 / approach_msg.angular.z;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));

    approach_msg.linear.x = 0.0;
    approach_msg.linear.y = 0.0;
    approach_msg.linear.z = 0.0;
    approach_msg.angular.x = 0.0;
    approach_msg.angular.y = 0.0;
    approach_msg.angular.z = 0.0;
    publish_cmd_vel_msg(approach_msg);
  }

  task_result_ = true;
  is_running_sub_task_thread_ = false;
}

TASK::STATUS TaskManager::turn(const geometry_msgs::Pose &target_pose)
{
   turn_to_target(target_pose);

   bool continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
   if(continue_result == false)
   {
     on_stop_task();
     return TASK::STOP;
   }

   return task_result_ ? TASK::SUCCESS : TASK::FAIL;
}

void TaskManager::turn_to_target(const std::string &target_name)
{
  double target_yaw = 0.0;

  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it == marker_name_list_.end())
      return;

    geometry_msgs::Pose target_pose, present_pose;

    std::string base_frame_id = robot_name_ + "/base_footprint";

    bool result = get_target_pose(marker_name, target_pose) &&
        get_target_pose(base_frame_id, present_pose);

    if(result == false)
    {
      ROS_ERROR("Couldn't find the target or present footprint");
      return;
    }

    double roll, pitch, yaw;
    get_euler_angle(present_pose.orientation, roll, pitch, yaw);

    Eigen::Vector3d present_position, target_position;

    tf::pointMsgToEigen(present_pose.position, present_position);
    tf::pointMsgToEigen(target_pose.position, target_position);

    Eigen::Vector3d target_vec = target_position - present_position;

    // if (x > y)
    if(fabs(target_vec.coeff(0)) > fabs(target_vec.coeff(1)))
    {
      if(target_vec.coeff(0) > 0)
        target_yaw = 0;
      else
        target_yaw = M_PI;
    }
    else
    {
      if(target_vec.coeff(1) > 0)
        target_yaw = M_PI * 0.5;
      else
        target_yaw = -M_PI * 0.5;
    }

    target_yaw = target_yaw - yaw;
  }

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::turn_to_target_thread, this, target_yaw));
  delete moving_thread_;
}

void TaskManager::turn_to_target(const geometry_msgs::Pose &target_pose)
{
  double target_yaw;
  geometry_msgs::Pose present_pose;

  std::string base_frame_id = robot_name_ + "/base_footprint";

  bool result = get_target_pose(base_frame_id, present_pose);

  if(result == false)
  {
    ROS_ERROR("Couldn't find the present footprint");
    task_result_ = false;
    return;
  }

  double roll, pitch, yaw;
  get_euler_angle(present_pose.orientation, roll, pitch, yaw);

  Eigen::Vector3d present_position, target_position;

  tf::pointMsgToEigen(present_pose.position, present_position);
  tf::pointMsgToEigen(target_pose.position, target_position);

  Eigen::Vector3d target_vec = target_position - present_position;

  // if (x > y) --> align x-axis
  if(fabs(target_vec.coeff(0)) > fabs(target_vec.coeff(1)))
  {
    if(target_vec.coeff(0) > 0)
      target_yaw = 0;
    else
      target_yaw = M_PI;
  }
  else
  {
    if(target_vec.coeff(1) > 0)
      target_yaw = M_PI_2;
    else
      target_yaw = -M_PI_2;
  }

  target_yaw = target_yaw - yaw;
  if(target_yaw > M_PI)
    target_yaw -= 2 * M_PI;
  if(target_yaw < - M_PI)
    target_yaw += 2 * M_PI;

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::turn_to_target_thread, this, target_yaw));
  delete moving_thread_;
}

void TaskManager::turn_to_target_thread(double yaw)
{
  is_running_sub_task_thread_ = true;

  if(yaw == 0)
  {
    task_result_ = true;
    return;
  }

  // turn to yaw
  geometry_msgs::Twist approach_msg;
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = (yaw > 0) ? leave_angular_vel_ : - leave_angular_vel_;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  int moving_time = yaw * 1000 / approach_msg.angular.z;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  task_result_ = true;
  is_running_sub_task_thread_ = false;
}

TASK::STATUS TaskManager::navigation(const std::string &target_name, bool recovery = false, int time_out = 0)
{
  return navigation(target_name, "", recovery, time_out);
}

TASK::STATUS TaskManager::navigation(const std::string &target_name, const std::string &real_target, bool recovery = false, int time_out = 0)
{
  int retry_times = 3;

  for(int ix = 0; ix < retry_times; ix++)
  {
    bool result = nav_to_target(target_name, real_target);

    if(result == false && recovery == true)
    {
      ROS_WARN_STREAM("Failed to find nav goal : " << target_name);

      // go to start point
      nav_to_target("nav_start", target_name);

      bool continue_result = sleep_for(SLEEP_MS, 0, is_running_sub_task_thread_, is_pause_, is_stop_);
      if(continue_result == false)
      {
        TASK::STATUS nav_result = is_stop_ ? TASK::STOP : TASK::TIME_OUT;
        cancel_nav();

        return nav_result;
      }

      // find object again
      // wait for detecting ar marker
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS * 10));

      // nav to target again
      result = nav_to_target(target_name, real_target);
      if(result == false)
        return TASK::FAIL;
    }

    bool continue_result = sleep_for(SLEEP_MS, 0, is_running_sub_task_thread_, is_pause_, is_stop_, time_out);
    if(continue_result == false)
    {
      TASK::STATUS nav_result = is_stop_ ? TASK::STOP : TASK::TIME_OUT;
      cancel_nav();

      return nav_result;
    }

    // check distance
    if(real_target == "")
    {
      std::size_t pos = target_name.find("ar_marker");
      if(pos != std::string::npos)
      {
        std::string base_frame_id = robot_name_ + "/base_footprint";
        double max_distance = 0.5;
        bool distance_result = check_distance(base_frame_id, target_name, max_distance);

        if(distance_result == true)
          return TASK::SUCCESS;
      }
      else
        return TASK::SUCCESS;
    }
    else
      return TASK::SUCCESS;
  }

  return TASK::FAIL;
}

TASK::STATUS TaskManager::navigation(const geometry_msgs::Pose &target_pose, bool recovery = false, int time_out = 0)
{
  return navigation(target_pose, "", recovery, time_out);
}

TASK::STATUS TaskManager::navigation(const geometry_msgs::Pose &target_pose, const std::string &real_target, bool recovery = false, int time_out = 0)
{
  int retry_times = 3;

  for(int ix = 0; ix < retry_times; ix++)
  {
    bool result = nav_to_target(target_pose, real_target);

    if(result == false && recovery == true)
    {
      // go to start point
      nav_to_target("nav_start");

      bool continue_result = sleep_for(SLEEP_MS, 0, is_running_sub_task_thread_, is_pause_, is_stop_);
      if(continue_result == false)
      {
        TASK::STATUS nav_result = is_stop_ ? TASK::STOP : TASK::TIME_OUT;
        cancel_nav();

        return nav_result;
      }

      // find object again
      // wait for detecting ar marker
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS * 10));

      // nav to target again
      result = nav_to_target(target_pose, real_target);
      if(result == false)
        return TASK::FAIL;
    }

    bool continue_result = sleep_for(SLEEP_MS, 0, is_running_sub_task_thread_, is_pause_, is_stop_, time_out);
    if(continue_result == false)
    {
      TASK::STATUS nav_result = is_stop_ ? TASK::STOP : TASK::TIME_OUT;
      cancel_nav();

      return nav_result;
    }


    // check distance
    if(real_target == "")
    {
      std::string base_frame_id = robot_name_ + "/base_footprint";
      double max_distance = 0.5;
      bool distance_result = check_distance(base_frame_id, target_pose, max_distance);

      if(distance_result == true)
        return TASK::SUCCESS;
    }
    else
      return TASK::SUCCESS;

  }

  return TASK::FAIL;
}

void TaskManager::cancel_nav()
{
  actionlib_msgs::GoalID cancel_msg;
  cancel_msg.id = navigation_goal_id_;

  cancel_nav_pub_.publish(cancel_msg);

  ROS_INFO("Nav goal is canceled.");
}

bool TaskManager::nav_to_target(const std::string& target_name)
{
  return nav_to_target(target_name, "");
}

bool TaskManager::nav_to_target(const std::string& target_name, const std::string& real_target)
{
  // go ar_marker
  geometry_msgs::Pose *target_pose = nullptr;

  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      target_pose = new geometry_msgs::Pose;

      bool result = get_target_pose(marker_name, *target_pose);
      if(result == false)
      {
        ROS_WARN_COND(DEBUG, "Failed to find correct ar marker.");

        target_pose = nullptr;

        task_result_ = false;

        return true;
      }

      Eigen::Vector3d offset(0, 0, 0.35);

      Eigen::Quaterniond target_orientation;
      Eigen::Vector3d object_position, target_position;
      Eigen::VectorXd global_offset;

      // target position
      tf::quaternionMsgToEigen(target_pose->orientation, target_orientation);
      tf::pointMsgToEigen(target_pose->position, object_position);
      global_offset = target_orientation.toRotationMatrix() * offset;
      global_offset.coeffRef(2) = 0.0;
      target_position = object_position + global_offset;

      tf::pointEigenToMsg(target_position, target_pose->position);

      // target orientation : global yaw
      double yaw = atan2(-global_offset.coeff(1), -global_offset.coeff(0));

      get_quaternion(0.0, 0.0, yaw, target_pose->orientation);
    }
  }

  // test code
  if(target_name == "nav_1")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.8;
    target_pose->position.y = 0.39;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;
  }

  if(target_name == "nav_1_goal")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.95;
    target_pose->position.y = 0.95;
    target_pose->position.z = 0;

    geometry_msgs::Quaternion orientation;
    get_quaternion(0, 0, M_PI * 0.25, orientation);
    target_pose->orientation = orientation;
  }

  if(target_name == "nav_2")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.8;
    target_pose->position.y = 0.13;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;
  }

  if(target_name == "nav_2_goal")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.95;
    target_pose->position.y = -0.95;
    target_pose->position.z = 0;

    geometry_msgs::Quaternion orientation;
    get_quaternion(0, 0, -M_PI * 0.25, orientation);
    target_pose->orientation = orientation;
  }

  if(target_name == "nav_start")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0;
    target_pose->position.y = 0;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;
  }

  if(target_pose != nullptr)
  {
    moving_thread_ = new boost::thread(boost::bind(&TaskManager::nav_to_target_thread, this, *target_pose, real_target));
    delete moving_thread_;
  }

  return true;
}

bool TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose)
{
  return nav_to_target(target_pose, "");
}

bool TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose, const std::string& real_target)
{
  moving_thread_ = new boost::thread(boost::bind(&TaskManager::nav_to_target_thread, this, target_pose, real_target));
  delete moving_thread_;

  return true;
}

void TaskManager::nav_to_target_thread(const geometry_msgs::Pose& target_pose, const std::string& real_target)
{
  is_running_sub_task_thread_ = true;

  geometry_msgs::PoseStamped nav_msg;
  nav_msg.header.stamp = ros::Time::now();
  nav_msg.header.frame_id = "map";
  nav_msg.pose = target_pose;

  publish_goal_nav_msg(nav_msg);

  // wait for accept
  while(navigation_status_ != actionlib_msgs::GoalStatus::ACTIVE)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS));

  ROS_INFO_STREAM_COND(DEBUG, "Navigation message is accepted. : " << navigation_status_);

  // wait for finishing navigation
  geometry_msgs::Pose real_target_pose;
  while(navigation_status_ == actionlib_msgs::GoalStatus::ACTIVE)
  {
    if(real_target != "")
    {
      std::string base_frame_id = robot_name_ + "/base_footprint";
      double max_distance = 1.0;
      bool distance_result = check_distance(base_frame_id, real_target, max_distance);

      if(distance_result == true)
      {
        cancel_nav();
        task_result_ = true;
        break;
      }
    }

    boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS));
  }

  task_result_ = true;

  ROS_INFO_STREAM_COND(DEBUG, "Navigation is finished. : " << navigation_status_);

  is_running_sub_task_thread_ = false;
}

TASK::STATUS TaskManager::find_target(const std::string &target_name)
{
  look_around(target_name);

  bool continue_result = sleep_for(SLEEP_MS, SLEEP_MS * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return TASK::STOP;
  }

  return task_result_ ? TASK::SUCCESS : TASK::FAIL;
}

void TaskManager::look_around(const std::string& target_name)
{
  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      moving_thread_ = new boost::thread(boost::bind(&TaskManager::look_around_thread, this, 1, marker_name));
      delete moving_thread_;
    }
  }
}

void TaskManager::look_around_thread(int direction, const std::string& target_name)
{
  if(target_name == "")
  {
    ROS_ERROR("No target name");
    return;
  }

  is_running_sub_task_thread_ = true;
  task_result_ = false;

  double linear_vel = 0.0;
  double angular_vel = 0.5;

  // turn around
  geometry_msgs::Twist approach_msg;
  approach_msg.linear.x = linear_vel;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = angular_vel * direction;
  publish_cmd_vel_msg(approach_msg);

  // check to find the target object
  geometry_msgs::Pose target_pose;

  int moving_time = 2 * M_PI * 1000 / angular_vel;
  int sleep_time = 200; //ms
  int total_moving_index = moving_time / sleep_time;

  for(int ix = 0; ix < total_moving_index; ix++)
  {
    if(is_stop_ == true)
      break;

    if(get_target_pose(target_name, target_pose))
    {
      ROS_INFO("Success to find the target object");
      task_result_ = true;
      break;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_time));
  }

  // stop moving
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  is_running_sub_task_thread_ = false;
}

bool TaskManager::check_distance(const std::string& from, const std::string& to, double max_distance)
{
  geometry_msgs::Pose target_pose, present_pose;

  bool result = get_target_pose(to, target_pose) &&
      get_target_pose(from, present_pose);

  if(result == false)
    return false;

  double diff_x = target_pose.position.x - present_pose.position.x;
  double diff_y = target_pose.position.y - present_pose.position.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  if(distance <= max_distance)
    return true;

  return false;
}

bool TaskManager::check_distance(const std::string& from, const geometry_msgs::Pose& to, double max_distance)
{
  geometry_msgs::Pose present_pose;

  bool result = get_target_pose(from, present_pose);

  if(result == false)
    return false;

  double diff_x = to.position.x - present_pose.position.x;
  double diff_y = to.position.y - present_pose.position.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);

  if(distance <= max_distance)
    return true;

  return false;
}

}
