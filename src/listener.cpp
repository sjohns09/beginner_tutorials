/** @file listener.cpp
 * @brief This tutorial demonstrates the simple receiving of messages over the ROS system.
 *
 * @author Samantha Johnson
 * @date November 7, 2017
 * @copyright (c) 2017, Samantha Johnson
 * @license BSD 3-Clause License
 *
 * @details This is the node that subscribes for messages on a topic.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

  /**
   * @brief method that displays the string that was heard by
   * the listener and returns an error message if that string
   * is odd.
   * @param Input is the message that is received on the topic
   */
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
