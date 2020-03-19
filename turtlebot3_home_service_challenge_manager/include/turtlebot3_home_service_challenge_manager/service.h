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

#ifndef TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_SERVICE_H
#define TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_SERVICE_H

#include <string>
#include <vector>

namespace turtlebot3_home_service_challenge
{

class Service
{
public:
  // enum

  // const

  // constructor
  Service(std::string name);

  // method
  std::string get_name() { return name_; }
  void set_object(const std::string& object_marker);
  void set_target(const std::string& target_marker);
  void set_object_position(const std::vector<double>& position);
  void set_object_position(double pos_x, double pos_y, double pos_z);
  void set_target_position(const std::vector<double>& position);
  void set_target_position(double pos_x, double pos_y, double pos_z);
  void set_room_x(double x_max, double x_min);
  void set_room_y(double y_max, double y_min);
  std::string get_object() {return object_marker_;}
  std::string get_target() {return target_marker_;}
  bool get_room_center(double& x, double&y);
  bool get_object_position(double& pos_x, double& pos_y, double& pos_z);
  bool get_object_position(std::vector<double>& position);
  bool get_target_position(double& pos_x, double& pos_y, double& pos_z);
  bool get_target_position(std::vector<double>& position);

  // variable

protected:

  // enum

  // const

  // method

  // variable
  std::string name_;
  std::string object_marker_;
  std::string target_marker_;
  std::vector<double> object_position_;
  std::vector<double> target_position_;
  std::vector<double> room_x_;
  std::vector<double> room_y_;

};
}

#endif // TURTLEBOT3_HOME_SERVICE_CHALLENGE_MANAGER_SERVICE_H
