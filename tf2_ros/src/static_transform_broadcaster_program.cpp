/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster_node.hpp"

int main(int argc, char ** argv)
{
  //Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;

  if (args.size() != 9 && args.size() != 10) {
    printf("A command line utility for manually sending a transform.\n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id \n");
    RCUTILS_LOG_ERROR(
      "static_transform_publisher exited due to not having the right number of arguments");
    return 2;
  }
  double x = std::stod(args[1]);
  double y = std::stod(args[2]);
  double z = std::stod(args[3]);
  double rx, ry, rz, rw;
  std::string frame_id, child_id;

  if (args.size() == 9) {
    // grab parameters from yaw, pitch, roll
    tf2::Quaternion quat;
    quat.setRPY(std::stod(args[6]), std::stod(args[5]), std::stod(args[4]));
    rx = quat.x();
    ry = quat.y();
    rz = quat.z();
    rw = quat.w();
    frame_id = args[7];
    child_id = args[8];
  } else {
    // quaternion supplied directly
    rx = std::stod(args[4]);
    ry = std::stod(args[5]);
    rz = std::stod(args[6]);
    rw = std::stod(args[7]);
    frame_id = args[8];
    child_id = args[9];
  }

  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", x},
    {"translation.y", y},
    {"translation.z", z},
    {"rotation.x", rx},
    {"rotation.y", ry},
    {"rotation.z", rz},
    {"rotation.w", rw},
    {"frame_id", frame_id},
    {"child_frame_id", child_id},
  });

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  RCLCPP_INFO(
    node->get_logger(), "Spinning until killed publishing transform from '%s' to '%s'",
    frame_id.c_str(), child_id.c_str());
  rclcpp::spin(node);
  return 0;
}
