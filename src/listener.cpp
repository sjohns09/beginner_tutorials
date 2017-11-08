/** @file listener.cpp
 * @brief This tutorial demonstrates the simple receiving of messages over the ROS system.
 *
 * @author Samantha Johnson
 * @date October 31, 2017
 * @copyright (c) 2017, Samantha Johnson
 *
 * @details This is the node that subscribes for messages on a topic.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg) {

  ROS_INFO("I heard: %s", msg->data.c_str());

  if (msg->data.at(0) == '.' ) {
    ROS_ERROR("Can't interpret a message from the upside-down!");
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
