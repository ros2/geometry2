/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

#include <utility>
#include <memory>
#include <stdexcept>
#include <string>
#include <random>

#include "tf2_ros/static_transform_broadcaster_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
std::string get_unique_node_name()
{
  const static std::string chars = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
  static std::random_device rd;
  static std::minstd_rand g{rd()};

  // The uniform_int_distribution takes a closed interval of [a, b]; since that
  // would include the \0, we remove one from our string length.
  static std::uniform_int_distribution<std::string::size_type> pick(0, chars.length() - 1);

  std::string s{"static_transform_publisher_"};

  size_t orig_length = s.length();
  s.resize(orig_length + 16);

  for (size_t i = orig_length; i < s.length(); ++i) {
    s[i] = chars[pick(g)];
  }

  return s;
}
}

namespace tf2_ros
{
StaticTransformBroadcasterNode::StaticTransformBroadcasterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node(get_unique_node_name().c_str(), options)
// TODO(clalancette): Anonymize the node name like it is in ROS1.
{
  geometry_msgs::msg::TransformStamped tf_msg;
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;

  tf_msg.header.stamp = this->now();
  tf_msg.transform.translation.x = this->declare_parameter("/translation/x", 0.0, descriptor);
  tf_msg.transform.translation.y = this->declare_parameter("/translation/y", 0.0, descriptor);
  tf_msg.transform.translation.z = this->declare_parameter("/translation/z", 0.0, descriptor);
  tf_msg.transform.rotation.x = this->declare_parameter("/rotation/x", 0.0, descriptor);
  tf_msg.transform.rotation.y = this->declare_parameter("/rotation/y", 0.0, descriptor);
  tf_msg.transform.rotation.z = this->declare_parameter("/rotation/z", 0.0, descriptor);
  tf_msg.transform.rotation.w = this->declare_parameter("/rotation/w", 1.0, descriptor);
  tf_msg.header.frame_id =
    this->declare_parameter("/frame_id", std::string("/frame"), descriptor);
  tf_msg.child_frame_id =
    this->declare_parameter("/child_frame_id", std::string("/child"), descriptor);

  // check frame_id != child_frame_id
  if (tf_msg.header.frame_id == tf_msg.child_frame_id) {
    RCLCPP_ERROR(this->get_logger(),
      "cannot publish static transform from '%s' to '%s', exiting",
      tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());
    throw std::runtime_error("child_frame_id cannot equal frame_id");
  }

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // send transform
  broadcaster_->sendTransform(tf_msg);
}
}  // namespace tf2_ros

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(tf2_ros::StaticTransformBroadcasterNode)
