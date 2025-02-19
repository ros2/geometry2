// Copyright 2025 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc.

#include "tf2_ros/managed_transform_buffer.hpp"

#include "tf2_ros/buffer_interface.h"

#include <tf2/LinearMath/Transform.hpp>

namespace tf2_ros
{

ManagedTransformBuffer::ManagedTransformBuffer(
  rclcpp::Clock::SharedPtr clock, tf2::Duration cache_time)
: clock_(clock)
{
  options_.start_parameter_event_publisher(false);
  options_.start_parameter_services(false);
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_thread_ = std::make_unique<std::thread>(
    std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, executor_.get()));
  registerAsUnknown();
  static_tf_buffer_ = std::make_unique<TFMap>();
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(clock_, cache_time);
  tf_buffer_->setUsingDedicatedThread(true);
  random_engine_ = std::mt19937(std::random_device{}());
  dis_ = std::uniform_int_distribution<>(0, 0xFFFFFF);
}

ManagedTransformBuffer::~ManagedTransformBuffer()
{
  deactivateListener();
  executor_->cancel();
  if (executor_thread_->joinable()) {
    executor_thread_->join();
  }
}

TransformStamped ManagedTransformBuffer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  const rclcpp::Duration timeout) const
{
  return get_transform_(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout));
}

TransformStamped ManagedTransformBuffer::lookupTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration timeout) const
{
  return get_transform_(target_frame, source_frame, time, timeout);
}

bool ManagedTransformBuffer::canTransform(
  const std::string & target_frame, const std::string & source_frame,
  const tf2::TimePoint & time, const tf2::Duration timeout,
  std::string * errstr) const
{
  return can_transform_(target_frame, source_frame, time, timeout, errstr);
}

bool ManagedTransformBuffer::canTransform(
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & time, const rclcpp::Duration timeout,
  std::string * errstr) const
{
  return can_transform_(target_frame, source_frame, fromRclcpp(time), fromRclcpp(timeout), errstr);
}

bool ManagedTransformBuffer::isStatic()
{
  return tf_buffer_->hasStaticTFsRequestsOnly();
}

void ManagedTransformBuffer::setCreateTimerInterface(
  CreateTimerInterface::SharedPtr create_timer_interface) const
{
  tf_buffer_->setCreateTimerInterface(create_timer_interface);
}

void ManagedTransformBuffer::activateListener()
{
  if (!tf_listener_) {
    options_.arguments({"--ros-args", "-r", "__node:=" + generateUniqueNodeName()});
    node_ = std::make_unique<rclcpp::Node>("_", options_);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
    executor_->add_node(node_->get_node_base_interface());
  }
}

void ManagedTransformBuffer::deactivateListener()
{
  if (tf_listener_) {
    tf_listener_.reset();
    executor_->remove_node(node_->get_node_base_interface());
    node_.reset();
  }
}

void ManagedTransformBuffer::registerAsUnknown()
{
  get_transform_ = [this](
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration timeout) -> TransformStamped {
      auto res = getUnknownTransform(target_frame, source_frame, time, timeout);
      return res;
    };
  can_transform_ = [this](
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const tf2::Duration timeout, std::string * errstr) -> bool {
      auto res = canUnknownTransform(target_frame, source_frame, time, timeout, errstr);
      return res;
    };
}

void ManagedTransformBuffer::registerAsDynamic()
{
  get_transform_ = [this](
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration timeout) -> TransformStamped {
      if (!tf_listener_) {
        activateListener();
      }
      auto res = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
      return res;
    };
  can_transform_ = [this](
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time,
    const tf2::Duration timeout, std::string * errstr) -> bool {
      if (!tf_listener_) {
        activateListener();
      }
      auto res = tf_buffer_->canTransform(target_frame, source_frame, time, timeout, errstr);
      return res;
    };
}

tf2::Transform ManagedTransformBuffer::msgToTf2(const Transform & msg) const
{
  auto tf = tf2::Transform();
  tf.setOrigin(tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
  tf.setRotation(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w));
  return tf;
}

Transform ManagedTransformBuffer::tf2ToMsg(const tf2::Transform & tf2) const
{
  Transform out;
  out.translation.x = tf2.getOrigin().getX();
  out.translation.y = tf2.getOrigin().getY();
  out.translation.z = tf2.getOrigin().getZ();
  out.rotation.w = tf2.getRotation().getW();
  out.rotation.x = tf2.getRotation().getX();
  out.rotation.y = tf2.getRotation().getY();
  out.rotation.z = tf2.getRotation().getZ();
  return out;
}

std::string ManagedTransformBuffer::generateUniqueNodeName()
{
  std::stringstream sstream;
  sstream << "managed_tf_listener_impl_" << std::hex << dis_(random_engine_)
          << dis_(random_engine_);
  return sstream.str();
}

std::optional<TransformStamped> ManagedTransformBuffer::getStaticTransform(
  const std::string & target_frame, const std::string & source_frame)
{
  auto key = std::make_pair(target_frame, source_frame);
  auto key_inv = std::make_pair(source_frame, target_frame);

  // Check if the transform is already in the buffer
  auto it = static_tf_buffer_->find(key);
  if (it != static_tf_buffer_->end()) {
    auto tf_msg = it->second;
    tf_msg.header.stamp = clock_->now();
    return std::make_optional<TransformStamped>(tf_msg);
  }

  // Check if the inverse transform is already in the buffer
  auto it_inv = static_tf_buffer_->find(key_inv);
  if (it_inv != static_tf_buffer_->end()) {
    auto tf_msg = it_inv->second;
    auto tf = msgToTf2(tf_msg.transform);
    tf2::Transform inv_tf = tf.inverse();
    TransformStamped inv_tf_msg;
    inv_tf_msg.transform = tf2ToMsg(inv_tf);
    inv_tf_msg.header.frame_id = tf_msg.child_frame_id;
    inv_tf_msg.child_frame_id = tf_msg.header.frame_id;
    inv_tf_msg.header.stamp = clock_->now();
    static_tf_buffer_->emplace(key, inv_tf_msg);
    return std::make_optional<TransformStamped>(inv_tf_msg);
  }

  // Check if transform is needed
  if (target_frame == source_frame) {
    auto tf_identity = tf2::Transform::getIdentity();
    TransformStamped tf_msg;
    tf_msg.transform = tf2ToMsg(tf_identity);
    tf_msg.header.frame_id = target_frame;
    tf_msg.child_frame_id = source_frame;
    tf_msg.header.stamp = clock_->now();
    static_tf_buffer_->emplace(key, tf_msg);
    return std::make_optional<TransformStamped>(tf_msg);
  }

  return std::nullopt;
}

TransformStamped ManagedTransformBuffer::getUnknownTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration timeout)
{
  // Try to get transform from local static buffer
  auto static_tf = getStaticTransform(target_frame, source_frame);
  if (static_tf.has_value()) {
    return static_tf.value();
  }

  // Initialize TF listener and get transform
  activateListener();
  TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
  } catch (const tf2::TransformException & ex) {
    deactivateListener();
    throw ex;
  }

  // If TF is static, add it to the static buffer. Otherwise, switch to dynamic listener
  if (tf_buffer_->hasStaticTFsRequestsOnly()) {
    deactivateListener();
    auto key = std::make_pair(target_frame, source_frame);
    static_tf_buffer_->emplace(key, tf);
  } else {
    registerAsDynamic();
  }

  return tf;
}

bool ManagedTransformBuffer::canUnknownTransform(
  const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
  const tf2::Duration timeout, std::string * errstr)
{
  // Try to get transform from local static buffer
  auto static_tf = getStaticTransform(target_frame, source_frame);
  if (static_tf.has_value()) {
    return true;
  }

  // Initialize TF listener and check if transform is available
  activateListener();
  auto can_transform = tf_buffer_->canTransform(target_frame, source_frame, time, timeout, errstr);

  // Deactivate listener if TF is static or not found. Otherwise, switch to dynamic listener
  if (tf_buffer_->hasStaticTFsRequestsOnly()) {
    deactivateListener();
  } else {
    registerAsDynamic();
  }

  return can_transform;
}

}  // namespace tf2_ros
