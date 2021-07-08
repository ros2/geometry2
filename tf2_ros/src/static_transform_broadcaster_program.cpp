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

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster_node.hpp>

#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

std::unordered_map<std::string, std::string> _make_arg_map(std::vector<std::string> && args)
{
  std::unordered_map<std::string, std::string> ret;
  /* collect from [exe] --option1 value --option2 value ... --optionN value */
  for (size_t x = 1; x < args.size(); x += 2) {
    ret.emplace(std::move(args[x]), std::move(args[x + 1]));
  }
  return ret;
}

void _print_usage()
{
  const char * usage =
    "usage: static_transform_publisher [--x X] [--y Y] [--z Z] [--qx QX] [--qy QY]"
    "[--qz QZ] [--qw QW] [--roll ROLL] [--pitch PITCH] [--yaw YAW] --frame-id FRAME_ID"
    "--child-frame-id CHILD_FRAME_ID\n\n"
    "A command line utility for manually sending a static transform.\n\nIf no translation or"
    " orientation is provided, the identity transform will be published.\n\nThe translation offsets"
    " are in meters.\n\nThe rotation may be provided as with roll, pitch, yaw euler angles"
    " (radians), or as a quaternion.\n\n"
    "required arguments:\n"
    "  --frame-id FRAME_ID parent frame\n"
    "  --child-frame-id CHILD_FRAME_ID child frame id\n\n"
    "optional arguments:\n"
    "  --x X                 x component of a vector represention the translation\n"
    "  --y Y                 y component of a vector represention of the translation\n"
    "  --z Z                 z component of a vector represention of the translation\n"
    "  --qx QX               x component of a quaternion represention of the rotation\n"
    "  --qy QY               y component of a quaternion representation of the rotation\n"
    "  --qz QZ               z component of a quaternion representation of the rotation\n"
    "  --qw QW               w component of a quaternion representation of the rotation\n"
    "  --roll ROLL           roll component of an Euler representation of the rotation (RPY)\n"
    "  --pitch PITCH         pitch component of an Euler representation of the rotation (RPY)\n"
    "  --yaw YAW             yaw component of an Euler representation of the rotation (RPY)";
  printf("%s\n", usage);
}

tf2::Quaternion _get_rotation(const std::unordered_map<std::string, std::string> & args)
{
  tf2::Quaternion quat;
  bool found_quaternion = false;
  bool found_rpy = false;
  auto iter = args.find("--qx");
  if (iter != args.end()) {
    quat.setX(std::stod(iter->second));
    found_quaternion = true;
  }
  iter = args.find("--qy");
  if (iter != args.end()) {
    quat.setY(std::stod(iter->second));
    found_quaternion = true;
  }
  iter = args.find("--qz");
  if (iter != args.end()) {
    quat.setZ(std::stod(iter->second));
    found_quaternion = true;
  }
  iter = args.find("--qw");
  if (iter != args.end()) {
    quat.setW(std::stod(iter->second));
    found_quaternion = true;
  }
  /* otherwise, look for roll, pitch, yaw */
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  iter = args.find("--roll");
  if (iter != args.end()) {
    roll = std::stod(iter->second);
    found_rpy = true;
  }
  iter = args.find("--pitch");
  if (iter != args.end()) {
    pitch = std::stod(iter->second);
    found_rpy = true;
  }
  iter = args.find("--yaw");
  if (iter != args.end()) {
    yaw = std::stod(iter->second);
    found_rpy = true;
  }
  if (found_quaternion && found_rpy) {
    RCUTILS_LOG_ERROR("cannot mix euler and quaternion arguments");
    _print_usage();
    exit(1);
  } else if (!found_quaternion) {
    quat.setRPY(roll, pitch, yaw);
  }
  return quat;
}

tf2::Vector3 _get_translation(const std::unordered_map<std::string, std::string> & args)
{
  tf2::Vector3 trans;
  auto iter = args.find("--x");
  if (iter != args.end()) {
    trans.setX(std::stod(iter->second));
  }
  iter = args.find("--y");
  if (iter != args.end()) {
    trans.setY(std::stod(iter->second));
  }
  iter = args.find("--z");
  if (iter != args.end()) {
    trans.setZ(std::stod(iter->second));
  }
  return trans;
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  std::unordered_map<std::string, std::string> arg_map = _make_arg_map(std::move(args));
  rclcpp::NodeOptions options;
  std::shared_ptr<tf2_ros::StaticTransformBroadcasterNode> node;
  tf2::Quaternion rotation;
  tf2::Vector3 translation;
  try {
    rotation = _get_rotation(arg_map);
    translation = _get_translation(arg_map);
  } catch (std::invalid_argument & e) {
    RCUTILS_LOG_ERROR("error parsing command line arguments");
    _print_usage();
    return 1;
  }
  std::string frame_id, child_id;
  auto iter = arg_map.find("--frame-id");
  if (iter == arg_map.end()) {
    _print_usage();
    return 1;
  }
  frame_id = iter->second;
  iter = arg_map.find("--child-frame-id");
  if (iter == arg_map.end()) {
    _print_usage();
    return 1;
  }
  child_id = iter->second;

  // override default parameters with the desired transform
  options.parameter_overrides(
  {
    {"translation.x", translation.x()},
    {"translation.y", translation.y()},
    {"translation.z", translation.z()},
    {"rotation.x", rotation.x()},
    {"rotation.y", rotation.y()},
    {"rotation.z", rotation.z()},
    {"rotation.w", rotation.w()},
    {"frame_id", frame_id},
    {"child_frame_id", child_id},
  });

  node = std::make_shared<tf2_ros::StaticTransformBroadcasterNode>(options);

  RCLCPP_INFO(
    node->get_logger(),
    "Spinning until killed publishing transform\ntranslation: ('%lf', '%lf', '%lf')\n"
    "rotation: ('%lf', '%lf', '%lf', '%lf')\nfrom '%s' to '%s'",
    translation.x(), translation.y(), translation.z(),
    rotation.x(), rotation.y(), rotation.z(), rotation.w(),
    frame_id.c_str(), child_id.c_str());
  rclcpp::spin(node);
  return 0;
}
