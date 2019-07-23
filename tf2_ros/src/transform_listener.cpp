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
#include <string>
#include <utility>

#include "tf2_ros/transform_listener.h"

#include "rclcpp/create_subscription.hpp"

using namespace tf2_ros;

// TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_INFO printf
#define ROS_WARN printf

TransformListener::TransformListener(tf2::BufferCore & buffer, bool spin_thread)
: optional_default_node_(rclcpp::Node::make_shared("transform_listener_impl")),
  buffer_(buffer)
{
  init(optional_default_node_, spin_thread);
}

TransformListener::~TransformListener()
{
  stop_thread_ = true;
}

void TransformListener::initThread(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface)
{
  stop_thread_ = false; // unused

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // This lambda is required because `std::thread` cannot infer the correct
  // rclcpp::spin, since there are more than one versions of it (overloaded).
  // see: http://stackoverflow.com/a/27389714/671658
  // I (wjwwood) chose to use the lamda rather than the static cast solution.
  auto run_func =
    [executor](rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface) {
      executor->add_node(node_base_interface);
      executor->spin();
      executor->remove_node(node_base_interface);
    };
  dedicated_listener_thread_ = thread_ptr(
    new std::thread(run_func, node_base_interface),
    [executor](std::thread * t) {
      executor->cancel();
      t->join();
      delete t;
      // TODO(tfoote) reenable callback queue processing
      // tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
    });
  // Tell the buffer we have a dedicated thread to enable timeouts
  buffer_.setUsingDedicatedThread(true);
}

void TransformListener::subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
{
  const tf2_msgs::msg::TFMessage & msg_in = *msg;
  // TODO(tfoote) find a way to get the authority
  std::string authority = "Authority undetectable";  // msg_evt.getPublisherName();  // lookup the authority
  for (auto i = 0u; i < msg_in.transforms.size(); i++) {
    try {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    } catch (tf2::TransformException & ex) {
      // /\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n",
        msg_in.transforms[i].child_frame_id.c_str(),
        msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
}
