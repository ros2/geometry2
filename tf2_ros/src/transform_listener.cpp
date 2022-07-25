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

/** \author Tully Foote */

#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "tf2_ros/transform_listener.h"

namespace tf2_ros
{

TransformListener::TransformListener(tf2::BufferCore & buffer, bool spin_thread)
: buffer_(buffer)
{
  // create a unique name for the node
  std::stringstream sstream;
  sstream << "transform_listener_impl_" << std::hex << reinterpret_cast<size_t>(this);
  rclcpp::NodeOptions options;
  // but specify its name in .arguments to override any __node passed on the command line
  options.arguments({"--ros-args", "-r", "__node:=" + std::string(sstream.str())});
  options.start_parameter_event_publisher(false);
  options.start_parameter_services(false);
  optional_default_node_ = rclcpp::Node::make_shared("_", options);
  init(
    optional_default_node_, spin_thread, DynamicListenerQoS(), StaticListenerQoS(),
    detail::get_default_transform_listener_sub_options(),
    detail::get_default_transform_listener_static_sub_options());
}

TransformListener::~TransformListener()
{
  if (spin_thread_) {
    executor_->cancel();
    dedicated_listener_thread_->join();
  }
}


inline
bool
_make_authority_str(
  const std::vector<rclcpp::TopicEndpointInfo> & endpoints,
  const rmw_gid_t & pub_gid,
  std::string & authority)
{
  for (const rclcpp::TopicEndpointInfo & pub : endpoints) {
    // Compare gid to see if this is the right publisher
    bool match = true;
    for (decltype(RMW_GID_STORAGE_SIZE) i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
      if (pub.endpoint_gid()[i] != pub_gid.data[i]) {
        match = false;
        break;
      }
    }
    if (match) {
      authority = pub.node_namespace() + '/' + pub.node_name();
      return true;
    }
  }
  return false;
}


std::string
TransformListener::make_authority_str(const rmw_gid_t & pub_gid, bool is_static)
{
  std::string authority = "Authority undetectable";
  std::vector<rclcpp::TopicEndpointInfo> & endpoints = tf_publishers_;
  if (is_static) {
    endpoints = tf_static_publishers_;
  }

  if (!_make_authority_str(endpoints, pub_gid, authority)) {
    // Update the cached endpoints
    tf_publishers_ = node_graph_interface_->get_publishers_info_by_topic(
      message_subscription_tf_->get_topic_name());
    tf_static_publishers_ = node_graph_interface_->get_publishers_info_by_topic(
      message_subscription_tf_static_->get_topic_name());

    // Try making the authority string again
    _make_authority_str(endpoints, pub_gid, authority);
  }

  return authority;
}


void TransformListener::subscription_callback(
  const tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
  const rclcpp::MessageInfo & msg_info,
  bool is_static)
{
  const tf2_msgs::msg::TFMessage & msg_in = *msg;
  const rmw_gid_t & pub_gid = msg_info.get_rmw_message_info().publisher_gid;

  std::string authority = make_authority_str(pub_gid, is_static);
  for (size_t i = 0u; i < msg_in.transforms.size(); i++) {
    try {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    } catch (const tf2::TransformException & ex) {
      // /\todo Use error reporting
      std::string temp = ex.what();
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "Failure to set received transform from %s to %s with error: %s\n",
        msg_in.transforms[i].child_frame_id.c_str(),
        msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
}

}  // namespace tf2_ros
