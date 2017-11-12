/** @file talker.cpp
 * @brief This tutorial demonstrates the simple sending of
 * messages over the ROS system.
 *
 * @author Samantha Johnson
 * @date November 7, 2017
 * @license BSD 3-Clause License
 * @copyright (c) 2017, Samantha Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @details This is the node that publishes messages to a topic.
 * It also calls the service StringFlip on the message before it
 * publishes that message.
 */

#include <sstream>
#include <string>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/StringFlip.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::NodeHandle tfn;
  ros::NodeHandle private_node_handle_("~");

  tf::TransformBroadcaster br;
  tf::Transform transform;

  // --- Service Request ---
  double loopRate;

  private_node_handle_.param("loopRate", loopRate, static_cast<double>(1));
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::StringFlip>(
      "string_flip");
  beginner_tutorials::StringFlip srv;
  std::string flippedString;
  srv.request.input = "Something Strange is Coming...";

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(loopRate);

  int count = 0;
  while (ros::ok()) {

    // --- Broadcast tf ---
    transform.setOrigin(
        tf::Vector3(2.0 * sin(ros::Time::now().toSec()),
                    2.0 * cos(ros::Time::now().toSec()), 0.0));

    tf::Quaternion q;
    q.setEuler(count, count+count, -count);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "world", "talk"));

    // -----Service Request -------
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

