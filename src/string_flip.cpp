/*
 * StringFlipClass.cpp
 *
 *  Created on: Nov 5, 2017
 *      Author: sammie
 */

#include <sstream>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/StringFlip.h"

bool string_flip(beginner_tutorials::StringFlip::Request &req,
                             beginner_tutorials::StringFlip::Response &res) {
  std::string str = req.input;

  ROS_INFO("Flipping String");

  reverse(str.begin(), str.end());
  res.output = str;

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "string_flip_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("string_flip", string_flip);
  ROS_INFO("Ready to flip string!");

  ros::spin();

  return 0;
}
