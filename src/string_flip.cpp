/** @file string_flip.cpp
 * @brief This service flips the contents of a string.
 *
 * @author Samantha Johnson
 * @date November 7, 2017
 * @copyright (c) 2017, Samantha Johnson
 * @license BSD 3-Clause License
 *
 * @details This service takes a string as a request, reverses the contents
 * of that string, and then returns the flipped string as the response.
 */

#include <sstream>
#include <bits/stdc++.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/StringFlip.h"

  /**
   * @brief A method that reverses the characters in a request string
   * and sets the response to this flipped string
   * @param Input is the request which is a string
   * @param Input is the response which is a string
   * @return Output is a boolean showing completion of request
   */
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
