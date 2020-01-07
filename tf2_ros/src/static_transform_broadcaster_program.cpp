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
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/static_transform_broadcaster_node.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "rcutils/logging_macros.h"

//TODO(clalancette re-enable this)
// bool validateXmlRpcTf(XmlRpc::XmlRpcValue tf_data) {
//   // Validate a TF stored in XML RPC format: ensures the appropriate fields
//   // exist. Note this does not check data types.
//   return tf_data.hasMember("child_frame_id") &&
//          tf_data.hasMember("header") &&
//          tf_data["header"].hasMember("frame_id") &&
//          tf_data.hasMember("transform") &&
//          tf_data["transform"].hasMember("translation") &&
//          tf_data["transform"]["translation"].hasMember("x") &&
//          tf_data["transform"]["translation"].hasMember("y") &&
//          tf_data["transform"]["translation"].hasMember("z") &&
//          tf_data["transform"].hasMember("rotation") &&
//          tf_data["transform"]["rotation"].hasMember("x") &&
//          tf_data["transform"]["rotation"].hasMember("y") &&
//          tf_data["transform"]["rotation"].hasMember("z") &&
//          tf_data["transform"]["rotation"].hasMember("w");
// };

int main(int argc, char ** argv)
{
  //Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions options;
  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;

  if (args.size() != 9 && args.size() != 10) {
    printf("A command line utility for manually sending a transform.\n");
    //printf("It will periodicaly republish the given transform. \n");
    printf("Usage: static_transform_publisher x y z qx qy qz qw frame_id child_frame_id \n");
    printf("OR \n");
    printf("Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id \n");
    //printf("OR \n");
    //printf("Usage: static_transform_publisher /param_name \n");
    //printf("\nThis transform is the transform of the coordinate frame from frame_id into the coordinate frame \n");
    //printf("of the child_frame_id.  \n");
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
    // grab parameters from roll, pitch, yaw
    tf2::Quaternion quat;
    quat.setRPY(std::stod(args[4]), std::stod(args[5]), std::stod(args[6]));
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
    {"/translation/x", x},
    {"/translation/y", y},
    {"/translation/z", z},
    {"/rotation/x", rx},
    {"/rotation/y", ry},
    {"/rotation/z", rz},
    {"/rotation/w", rw},
    {"/frame_id", frame_id},
    {"/child_frame_id", child_id},
  });

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  // else if (args.size() == 2) {
  //   const std::string param_name = args[1];
  //   ROS_INFO_STREAM("Looking for TF in parameter: " << param_name);
  //   XmlRpc::XmlRpcValue tf_data;

  //   if (!ros::param::has(param_name) || !ros::param::get(param_name, tf_data)) {
  //     ROS_FATAL_STREAM("Could not read TF from parameter server: " << param_name);
  //     return -1;
  //   }

  //   // Check that all required members are present & of the right type.
  //   if (!validateXmlRpcTf(tf_data)) {
  //     ROS_FATAL_STREAM("Could not validate XmlRpcC for TF data: " << tf_data);
  //     return -1;
  //   }

  //   msg.transform.translation.x = (double) tf_data["transform"]["translation"]["x"];
  //   msg.transform.translation.y = (double) tf_data["transform"]["translation"]["y"];
  //   msg.transform.translation.z = (double) tf_data["transform"]["translation"]["z"];
  //   msg.transform.rotation.x = (double) tf_data["transform"]["rotation"]["x"];
  //   msg.transform.rotation.y = (double) tf_data["transform"]["rotation"]["y"];
  //   msg.transform.rotation.z = (double) tf_data["transform"]["rotation"]["z"];
  //   msg.transform.rotation.w = (double) tf_data["transform"]["rotation"]["w"];
  //   msg.header.stamp = clock->now();
  //   msg.header.frame_id = (std::string) tf_data["header"]["frame_id"];
  //   msg.child_frame_id = (std::string) tf_data["child_frame_id"];
  // }
  RCLCPP_INFO(
    node->get_logger(), "Spinning until killed publishing transform from '%s' to '%s'",
    frame_id.c_str(), child_id.c_str());
  rclcpp::spin(node);
  return 0;
}
