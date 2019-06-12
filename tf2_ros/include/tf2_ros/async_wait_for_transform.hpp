// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2_ROS__ASYNC_WAIT_FOR_TRANSFORM_HPP_
#define TF2_ROS__ASYNC_WAIT_FOR_TRANSFORM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>

#include <future>
#include <memory>
#include <string>
#include <vector>

namespace tf2_ros
{
typedef std::function<void (const geometry_msgs::msg::TransformStamped &)> TransformCallback;

/// \brief Asyncronously wait for a transform (by polling).
/// \internal
///
/// \param[in] base_interface A node base interface for a node that will be executed while waiting
/// \param[in] timer_interface interface on the same node that will be executed
/// \param[in] clock a clock to get the current time for detecting a timeout
/// \param[in] buffer_ the tf2 buffer to use to lookup the transform
/// \param[in] target_frame frame to transform to
/// \param[in] source_frame frame to transform from
/// \param[in] timeout how long to wait for the transform
/// \param[in] callback a function to call when the timer is ready
/// \param[in] period how often to check if the transform is ready
/// \param[in] group the callback group to add the polling timer to
TF2_ROS_PUBLIC
std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>>
async_wait_for_transform(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timer_interface,
  rclcpp::Clock::SharedPtr clock,
  std::shared_ptr<tf2_ros::BufferInterface> buffer_,
  std::string target_frame,
  std::string source_frame,
  rclcpp::Time transform_time,
  rclcpp::Duration timeout,
  TransformCallback callback,
  rclcpp::Duration period,
  rclcpp::callback_group::CallbackGroup::SharedPtr group);

/// \brief Asyncronously wait for a transform (by polling).
///
/// The returned share_ptr must be kept alive while waiting for the transform.
/// The callback will only be called if the transform was successfully gotten.
/// A timeout will result in an exception on the returned future.
///
/// \param[in] node a node class that will be executed while waiting
/// \param[in] target_frame frame to transform to
/// \param[in] source_frame frame to transform from
/// \param[in] timeout how long to wait for the transform
/// \param[in] callback a function to call when the timer is ready
/// \param[in] period how often to check if the transform is ready
/// \param[in] group the callback group to add the polling timer to
/// \return a shared_ptr to a future that holds the result.
template<typename NodePtr>
std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>>
async_wait_for_transform(
  NodePtr node,
  std::shared_ptr<tf2_ros::BufferInterface> buffer,
  std::string target_frame,
  std::string source_frame,
  rclcpp::Time transform_time,
  rclcpp::Duration timeout,
  TransformCallback callback = nullptr,
  rclcpp::Duration period = std::chrono::milliseconds(10),  // 100Hz
  rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
{
  return async_wait_for_transform(
    node->get_node_base_interface(),
    node->get_node_timers_interface(),
    node->get_node_clock_interface()->get_clock(),
    buffer,
    target_frame,
    source_frame,
    transform_time,
    timeout,
    callback,
    period,
    group);
}
}  // namespace tf2_ros

#endif  // TF2_ROS__ASYNC_WAIT_FOR_TRANSFORM_HPP_
