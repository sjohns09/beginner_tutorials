/** @file string_flip.cpp
 * @brief This service flips the contents of a string.
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
 * @details This service takes a string as a request, reverses the contents
 * of that string, and then returns the flipped string as the response.
 */

#include <sstream>
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
