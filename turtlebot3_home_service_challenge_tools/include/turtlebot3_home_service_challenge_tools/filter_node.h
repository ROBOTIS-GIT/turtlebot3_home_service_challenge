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

#ifndef TURTLEBOT3_HOME_SERVICE_CHALLENGE_TOOLS_FILTER_NODE_H
#define TURTLEBOT3_HOME_SERVICE_CHALLENGE_TOOLS_FILTER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

const int MIN_MEAN_K = 2;
const int MAX_MEAN_K = 100;

int mean_k_;
double std_dev_mul_;
ros::Publisher filtered_scan_pub_;

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void publish_filtered_scan(const sensor_msgs::LaserScan &msg);


#endif // TURTLEBOT3_HOME_SERVICE_CHALLENGE_TOOLS_FILTER_NODE_H
