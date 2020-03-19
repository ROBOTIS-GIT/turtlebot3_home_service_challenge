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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "turtlebot3_home_service_challenge_manager/tb3_hsc_manager.h"

const int SPIN_RATE = 30;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "turtlebot3_home_service_challenge_manager");

  ros::NodeHandle nh(ros::this_node::getName());

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  turtlebot3_home_service_challenge::TaskManager *task_manager = new turtlebot3_home_service_challenge::TaskManager();

  ROS_INFO("Start task manager!");

  //node loop
  while (ros::ok())
  {
    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

