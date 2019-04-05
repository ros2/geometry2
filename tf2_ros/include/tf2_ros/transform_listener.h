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
  TransformListener(tf2::BufferCore & buffer, bool spin_thread = true);

  TF2_ROS_PUBLIC
  TransformListener(tf2::BufferCore & buffer, rclcpp::Node::SharedPtr nh, bool spin_thread = true);

  TF2_ROS_PUBLIC
  TransformListener(
    tf2::BufferCore & buffer,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
    const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr & node_topics,
    bool spin_thread = true);

  TF2_ROS_PUBLIC
  ~TransformListener();

private:
  /// Initialize this transform listener, subscribing, advertising services, etc.
  void init();
  void initThread();

  template<
    typename MessageT,
    typename CallbackT,
    typename Alloc = std::allocator<void>,
    typename SubscriptionT = rclcpp::Subscription<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>>
  std::shared_ptr<SubscriptionT>
  create_subscription(
    const std::string & topic_name,
    CallbackT && callback,
    const rmw_qos_profile_t & qos_profile = rmw_qos_profile_default,
    rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr,
    bool ignore_local_publications = false,
    typename rclcpp::message_memory_strategy::MessageMemoryStrategy<
      typename rclcpp::subscription_traits::has_message_type<CallbackT>::type, Alloc>::SharedPtr
    msg_mem_strat = nullptr,
    std::shared_ptr<Alloc> allocator = nullptr);

  /// Callback function for ros message subscriptoin
  void subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void static_subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
  void subscription_callback_impl(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static);

  // ros::CallbackQueue tf_message_callback_queue_;
  std::thread * dedicated_listener_thread_;
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  const rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;

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
