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

#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/visibility_control.h>

#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/qos.hpp>

#include <functional>
#include <memory>
#include <thread>
#include <utility>


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
    node_logging_interface_ = node->get_node_logging_interface();
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
    node_logging_interface_ = node->get_node_logging_interface();

    using callback_t = std::function<void (tf2_msgs::msg::TFMessage::SharedPtr)>;
    callback_t cb = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, false);
    callback_t static_cb = std::bind(
      &TransformListener::subscription_callback, this, std::placeholders::_1, true);

    message_subscription_tf_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
      node,
      "/tf",
      qos,
      std::move(cb),
      options);
    message_subscription_tf_static_ = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
      node,
      "/tf_static",
      static_qos,
      std::move(static_cb),
      static_options);

    if (spin_thread) {
      initThread(node->get_node_base_interface());
    }
  }

  TF2_ROS_PUBLIC
  void initThread(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface);

  /// Callback function for ros message subscriptoin
  TF2_ROS_PUBLIC
  void subscription_callback(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

  // ros::CallbackQueue tf_message_callback_queue_;
  using thread_ptr =
    std::unique_ptr<std::thread, std::function<void (std::thread *)>>;
  thread_ptr dedicated_listener_thread_;

  rclcpp::Node::SharedPtr optional_default_node_ = nullptr;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr message_subscription_tf_static_;
  tf2::BufferCore & buffer_;
  tf2::TimePoint last_update_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
};
}  // namespace tf2_ros

#endif  // TF2_ROS__TRANSFORM_LISTENER_H_
