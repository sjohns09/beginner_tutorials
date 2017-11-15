/** @file TutorialTests.cpp
 * @brief This contains tests for the beginner_tutorials package
 *
 * @author Samantha Johnson
 * @date November 14, 2017
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
 * @details This contains the level 2 integration tests for the talker
 * node and the StringFlip service
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "beginner_tutorials/StringFlip.h"
#include "std_msgs/String.h"


void callback1(const std_msgs::String::ConstPtr& msg) {
  EXPECT_EQ(msg->data.c_str(), "Something Strange is Coming...");
}

TEST(StringFlip_test, testListenForMessageFromTalkerNode) {
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1, callback1);
}

TEST(StringFilp_test, testCallStringFlipReturnsFlippedString) {
  ros::NodeHandle node;
  ros::ServiceClient client =
      node.serviceClient<beginner_tutorials::StringFlip>("string_flip");
  beginner_tutorials::StringFlip srv;
  srv.request.input = "FLIP";
  client.call(srv);
  std::string flippedString = srv.response.output;

  EXPECT_EQ(flippedString, "PILF");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "TutorialTests");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
