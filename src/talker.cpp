/** @file talker.cpp
 * @brief This tutorial demonstrates the simple sending of
 * messages over the ROS system.
 *
 * @author Samantha Johnson
 * @date November 7, 2017
 * @copyright (c) 2017, Samantha Johnson
 * @license BSD 3-Clause License
 *
 * @details This is the node that publishes messages to a topic.
 * It also calls the service StringFlip on the message before it
 * publishes that message.
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/StringFlip.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  double loopRate;

  ros::NodeHandle private_node_handle_("~");

  private_node_handle_.param("loopRate", loopRate, double(1));

  ros::ServiceClient client = n.serviceClient<beginner_tutorials::StringFlip>(
      "string_flip");

  beginner_tutorials::StringFlip srv;
  std::string flippedString;

  srv.request.input = "Something Strange is Coming...";

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(loopRate);

  int count = 0;
  while (ros::ok()) {

    if (client.call(srv)) {
      ROS_INFO("Calling Service StringFlip");
      flippedString = srv.response.output;
      srv.request.input = flippedString;
    } else {
      ROS_FATAL("Failed to call service string_flip");
      return 1;
    }

    std_msgs::String msg;

    std::stringstream ss;
    ss << flippedString << " [" << count << "]";
    msg.data = ss.str();

    ROS_INFO("Message to send: %s", msg.data.c_str());

    if (flippedString != "Something Strange is Coming...") {
      ROS_WARN("Sending a message from the upside-down");
    }

    chatter_pub.publish(msg);

    ros::spinOnce();

    ROS_DEBUG("Waiting to publish next message");
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

