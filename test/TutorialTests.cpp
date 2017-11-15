// Doxygen

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "beginner_tutorials/StringFlip.h"
#include "std_msgs/String.h"


void callback1(const std_msgs::String::ConstPtr& msg) {

  EXPECT_EQ(msg->data.c_str(),"Something Strange is Coming...");

}

TEST(StringFlip_test, testListenForMessageFromTalkerNode) {

  ros::NodeHandle n;

  std::string message;
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
