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

#include <functional>
#include <memory>
#include <thread>
#include <utility>

#include "tf2/buffer_core.h"
#include "tf2/time.h"
#include "tf2_ros/visibility_control.h"

#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/qos.hpp"

namespace tf2_ros
{

namespace detail
{
template<class AllocatorT = std::allocator<void>>
rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>
get_default_transform_listener_sub_options()
{
  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability};
  return options;
}

template<class AllocatorT = std::allocator<void>>
rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>
get_default_transform_listener_static_sub_options()
{
  rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions{
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability};
  return options;
}
}  // namespace detail

/** \brief This class provides an easy way to request and receive coordinate frame transform information.
 */
class TransformListener
{
public:
  /**@brief Constructor for transform listener */
  TF2_ROS_PUBLIC
  explicit TransformListener(tf2::BufferCore & buffer, bool spin_thread = true);

  template<class NodeT, class AllocatorT = std::allocator<void>>
  TransformListener(
    tf2::BufferCore & buffer,
    NodeT && node,
    bool spin_thread = true,
    const rclcpp::QoS & qos = DynamicListenerQoS(),
    const rclcpp::QoS & static_qos = StaticListenerQoS(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options =
    detail::get_default_transform_listener_sub_options<AllocatorT>(),
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options =
    detail::get_default_transform_listener_static_sub_options<AllocatorT>())
  : buffer_(buffer)
  {
    init(node, spin_thread, qos, static_qos, options, static_options);
  }

  TF2_ROS_PUBLIC
  virtual ~TransformListener();

private:
  template<class NodeT, class AllocatorT = std::allocator<void>>
  void init(
    NodeT && node,
    bool spin_thread,
    const rclcpp::QoS & qos,
    const rclcpp::QoS & static_qos,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & options,
    const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> & static_options)
  {
    spin_thread_ = spin_thread;
    node_base_interface_ = node->get_node_base_interface();
    node_logging_interface_ = node->get_node_logging_interface();

    using callback_t = std::function<void (tf2_msgs::msg::TFMessage::ConstSharedPtr)>;
    callback_t cb = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, true);

    if (spin_thread_) {
      // Create new callback group for message_subscription of tf and tf_static
      callback_group_ = node_base_interface_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
      // Duplicate to modify option of subscription
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_options = options;
      rclcpp::SubscriptionOptionsWithAllocator<AllocatorT> tf_static_options = static_options;
      tf_options.callback_group = callback_group_;
      tf_static_options.callback_group = callback_group_;

      message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node, "/tf", qos, std::move(cb), tf_options);
      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node, "/tf_static", static_qos, std::move(static_cb), tf_static_options);

      // Create executor with dedicated thread to spin.
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      dedicated_listener_thread_ = std::make_unique<std::thread>([&]() {executor_->spin();});
      // Tell the buffer we have a dedicated thread to enable timeouts
      buffer_.setUsingDedicatedThread(true);
    } else {
      message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node, "/tf", qos, std::move(cb), options);
      message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
        node, "/tf_static", static_qos, std::move(static_cb), static_options);
    }
  }
  /// Callback function for ros message subscriptoin
  TF2_ROS_PUBLIC
  void subscription_callback(tf2_msgs::msg::TFMessage::ConstSharedPtr msg, bool is_static);

  // ros::CallbackQueue tf_message_callback_queue_;
  bool spin_thread_{false};
  std::unique_ptr<std::thread> dedicated_listener_thread_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  rclcpp::Node::SharedPtr optional_default_node_ = nullptr;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  tf2::BufferCore & buffer_;
  tf2::TimePoint last_update_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
};
}  // namespace tf2_ros

#endif  // TF2_ROS__TRANSFORM_LISTENER_H_
