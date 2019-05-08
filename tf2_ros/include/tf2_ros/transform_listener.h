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

#ifndef TF2_ROS__TRANSFORM_LISTENER_H_
#define TF2_ROS__TRANSFORM_LISTENER_H_

#include <memory>
#include <string>
#include <thread>
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/visibility_control.h"


namespace tf2_ros
{

/** \brief This class provides an easy way to request and receive coordinate frame transform information.
 */
class TransformListener
{
public:
  /**@brief Constructor for transform listener */
  TF2_ROS_PUBLIC
  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true);

  TF2_ROS_PUBLIC
  TransformListener(tf2::BufferCore & buffer, rclcpp::Node::SharedPtr nh, bool spin_thread = true);

  template<class AllocatorT = std::allocator<void>>
  TransformListener(
    tf2::BufferCore & buffer,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
    bool spin_thread = true)
  : dedicated_listener_thread_(NULL),
    buffer_(buffer),
    using_dedicated_thread_(false)
  {
    using MessageT = tf2_msgs::msg::TFMessage;
    using CallbackT = std::function<void(const std::shared_ptr<MessageT>)>;
    using CallbackMessageT =
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type;
    using rclcpp::message_memory_strategy::MessageMemoryStrategy;

    CallbackT standard_callback = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, false);
    CallbackT static_callback = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, true);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 100;

    auto msg_mem_strat = MessageMemoryStrategy<MessageT, AllocatorT>::create_default();

    rclcpp::SubscriptionEventCallbacks callbacks;

    auto allocator = std::make_shared<AllocatorT>();

    message_subscription_tf_ =
      rclcpp::create_subscription<MessageT, CallbackT, AllocatorT, CallbackMessageT>(
        node_topics_interface.get(), "/tf", std::move(standard_callback), custom_qos_profile,
        callbacks, nullptr, false, false, msg_mem_strat, allocator);
    message_subscription_tf_static_ =
      rclcpp::create_subscription<MessageT, CallbackT, AllocatorT, CallbackMessageT>(
        node_topics_interface.get(), "/tf_static", std::move(static_callback), custom_qos_profile,
        callbacks, nullptr, false, false, msg_mem_strat, allocator);

    if (spin_thread) {
      initThread(node_base_interface);
    }
  }

  TF2_ROS_PUBLIC
  virtual ~TransformListener();

private:
  void initThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface);

  /// Callback function for ros message subscriptoin
  void subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

  // ros::CallbackQueue tf_message_callback_queue_;
  std::thread * dedicated_listener_thread_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  tf2::BufferCore & buffer_;
  bool using_dedicated_thread_;
  tf2::TimePoint last_update_;

  void dedicatedListenerThread()
  {
    while (using_dedicated_thread_) {
      break;
      // TODO(tfoote) reenable callback queue processing
      // tf_message_callback_queue_.callAvailable(ros::WallDuration(0.01));
    }
  }
};
}  // namespace tf2_ros

#endif  // TF2_ROS__TRANSFORM_LISTENER_H_
