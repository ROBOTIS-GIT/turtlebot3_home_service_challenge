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

#include "turtlebot3_home_service_challenge_tools/filter_node.h"

const int SPIN_RATE = 30;

//node main
int main(int argc, char **argv)
{

  //init ros
  ros::init(argc, argv, "laserscan_filter");

  ros::NodeHandle nh(ros::this_node::getName());
  ros::NodeHandle p_nh("~");

  std::string input_scan_name = p_nh.param<std::string>("input_topic", "/scan");
  std::string output_scan_name = p_nh.param<std::string>("output_topic", "/filtered_scan");
  mean_k_ = p_nh.param<int>("mean_k", 2);
  std_dev_mul_ = p_nh.param<double>("std_dev_mul", 1.0);

  ROS_INFO_STREAM("K : " << mean_k_ << ", Multiplier : " << std_dev_mul_);

  filtered_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>(output_scan_name, 0);
  ros::Subscriber laser_scan_sub = nh.subscribe(input_scan_name, 1, laser_scan_callback);

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  ROS_INFO("Start laser scan filter");


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

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan filterd_msg;
  filterd_msg = *msg;

  int laser_size = msg->ranges.size();

  for(int ix = 0; ix < laser_size; ix++)
  {
    double input_value = msg->ranges.at(ix);
    if(input_value < msg->range_min || input_value > msg->range_max)
      continue;

    int count = 0;
    double sum = 0;
    double squared_sum = 0;

    for(int neighbor_index = 0; neighbor_index <= mean_k_; neighbor_index++)
    {
      int index = (laser_size + ix - int(mean_k_ * 0.5) + neighbor_index) % laser_size;
      if(index == ix)
        continue;

      double value = msg->ranges.at(index);
      if(value < msg->range_min || value > msg->range_max)
        continue;

      count += 1;
      sum += value;
      squared_sum += (value * value);
    }

    //    if(count == 0 || count == 1)
    if(count == 0)
    {
      filterd_msg.ranges.at(ix) = 0.0;
      continue;
    }

    double mean = sum / count;
    double std_dev = sqrt((squared_sum / count - mean * mean));

    // outlier
    std::string del = "";
    if(input_value < (mean - (std_dev * std_dev_mul_)) || input_value > (mean + (std_dev * std_dev_mul_)))
    {
      filterd_msg.ranges.at(ix) = 0.0;
    }

  }

  publish_filtered_scan(filterd_msg);
}

void publish_filtered_scan(const sensor_msgs::LaserScan &msg)
{
  filtered_scan_pub_.publish(msg);
}
