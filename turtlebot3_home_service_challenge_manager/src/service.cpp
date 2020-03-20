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

#include "turtlebot3_home_service_challenge_manager/service.h"

namespace turtlebot3_home_service_challenge
{

Service::Service(std::string name)
  : name_(name)
{

}

void Service::set_object(const std::string& object_marker)
{
  object_marker_ = object_marker;
}

void Service::set_target(const std::string& target_marker)
{
  target_marker_ = target_marker;
}

void Service::set_object_position(const std::vector<double>& position)
{
  if(position.size() != 3)
    return;

  object_position_.clear();

  object_position_.push_back(position.at(0));
  object_position_.push_back(position.at(1));
  object_position_.push_back(position.at(2));
}

void Service::set_object_position(double pos_x, double pos_y, double pos_z)
{
  object_position_.clear();

  object_position_.push_back(pos_x);
  object_position_.push_back(pos_y);
  object_position_.push_back(pos_z);
}

void Service::set_target_position(const std::vector<double>& position)
{
  if(position.size() != 3)
    return;

  target_position_.clear();

  target_position_.push_back(position.at(0));
  target_position_.push_back(position.at(1));
  target_position_.push_back(position.at(2));
}

void Service::set_target_position(double pos_x, double pos_y, double pos_z)
{
  target_position_.clear();

  target_position_.push_back(pos_x);
  target_position_.push_back(pos_y);
  target_position_.push_back(pos_z);
}

void Service::set_room_x(double x_max, double x_min)
{
  room_x_.clear();

  room_x_.push_back(x_max);
  room_x_.push_back(x_min);
}

void Service::set_room_y(double y_max, double y_min)
{
  room_y_.clear();

  room_y_.push_back(y_max);
  room_y_.push_back(y_min);
}

bool Service::get_room_center(double& x, double&y)
{
  if(room_x_.size() != 2 || room_y_.size() != 2)
    return false;

  x = (room_x_.at(0) + room_x_.at(1)) * 0.5;
  y = (room_y_.at(0) + room_y_.at(1)) * 0.5;

  return true;
}

bool Service::get_object_position(double& pos_x, double& pos_y, double& pos_z)
{
  if(object_position_.size() != 3)
    return false;

  pos_x = object_position_.at(0);
  pos_y = object_position_.at(1);
  pos_z = object_position_.at(2);

  return true;
}

bool Service::get_object_position(std::vector<double>& position)
{
  if(object_position_.size() != 3)
    return false;

  position = object_position_;

  return true;
}

bool Service::get_target_position(double& pos_x, double& pos_y, double& pos_z)
{
  if(target_position_.size() != 3)
    return false;

  pos_x = target_position_.at(0);
  pos_y = target_position_.at(1);
  pos_z = target_position_.at(2);

  return true;
}

bool Service::get_target_position(std::vector<double>& position)
{
  if(target_position_.size() != 3)
    return false;

  position = target_position_;

  return true;
}

} // namespace
