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

#ifndef TF2_ROS__MANAGED_TRANSFORM_BUFFER_HPP_
#define TF2_ROS__MANAGED_TRANSFORM_BUFFER_HPP_

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Transform.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>

namespace std
{
template<>
struct hash<std::pair<std::string, std::string>>
{
  size_t operator()(const std::pair<std::string, std::string> & p) const
  {
    size_t h1 = std::hash<std::string>{}(p.first);
    size_t h2 = std::hash<std::string>{}(p.second);
    return h1 ^ (h2 << 1u);
  }
};
}  // namespace std

namespace tf2_ros
{
using Key = std::pair<std::string, std::string>;
struct PairEqual
{
  bool operator()(const Key & p1, const Key & p2) const
  {
    return p1.first == p2.first && p1.second == p2.second;
  }
};

using std::chrono_literals::operator""ms;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using TFMap = std::unordered_map<Key, TransformStamped, std::hash<Key>, PairEqual>;

/**
 * \brief A managed TF buffer that handles listener node lifetime. This buffer triggers listener
 * only for first occurrence of frames pair. After that, the local buffer is used for storing
 * static transforms. If a dynamic transform is detected, the listener is switched to dynamic mode
 * and acts as a regular TF buffer.
 */
class ManagedTransformBuffer
{
public:
  /**
   * \brief Construct a new Managed Transform Buffer object
   * \param clock A clock to use for time and sleeping
   * \param cache_time How long to keep a history of transforms
   */
  TF2_ROS_PUBLIC explicit ManagedTransformBuffer(
    rclcpp::Clock::SharedPtr clock,
    tf2::Duration cache_time = tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));

  /** \brief Destroy the Managed Transform Buffer object */
  ~ManagedTransformBuffer();

  /** \brief Get the transform between two frames by frame ID.
   * \param target_frame The frame to which data should be transformed
   * \param source_frame The frame where the data originated
   * \param time The time at which the value of the transform is desired. (0 will get the latest)
   * \param timeout How long to block before failing
   * \return The transform between the frames
   *
   * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
   * tf2::ExtrapolationException, tf2::InvalidArgumentException
   */
  TF2_ROS_PUBLIC TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const tf2::Duration timeout) const;

  /** \brief Get the transform between two frames by frame ID.
   * \sa lookupTransform(const std::string&, const std::string&, const tf2::TimePoint&,
   *                     const tf2::Duration)
   */
  TF2_ROS_PUBLIC TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0)) const;

  /** \brief Test if a transform is possible
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param target_time The time at which to transform
   * \param timeout How long to block before failing
   * \param errstr A pointer to a string which will be filled with why the transform failed, if not nullptr
   * \return True if the transform is possible, false otherwise
   */
  TF2_ROS_PUBLIC bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time, const tf2::Duration timeout,
    std::string * errstr = nullptr) const;

  /** \brief Test if a transform is possible
   * \sa canTransform(const std::string&, const std::string&,
   *                  const tf2::TimePoint&, const tf2::Duration, std::string*)
   */
  TF2_ROS_PUBLIC bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time,
    const rclcpp::Duration timeout = rclcpp::Duration::from_nanoseconds(0),
    std::string * errstr = nullptr) const;

  /** \brief Check if all TFs requests have been for static TF so far.
   * \return true if only static TFs have been requested
   */
  TF2_ROS_PUBLIC bool isStatic();

  /**
   * \brief Set timer interface for the tf2_ros::Buffer instance.
   * \param create_timer_interface the timer interface to set
   */
  TF2_ROS_PUBLIC void setCreateTimerInterface(
    CreateTimerInterface::SharedPtr create_timer_interface) const;

private:
  /** \brief Convert Transform msg to tf2.
   * It can not be included from tf2_geometry_msgs because of circular dependency.
   * \param msg the Transform message to convert
   * \return the converted tf2 transform
   */
  tf2::Transform msgToTf2(const Transform & msg) const;

  /** \brief Convert tf2 to Transform msg.
   * It can not be included from tf2_geometry_msgs because of circular dependency.
   * \param tf2 the tf2 transform to convert
   * \return the converted Transform message
   */
  Transform tf2ToMsg(const tf2::Transform & tf2) const;

  /** \brief Generate node name with unique suffix.
   * \return node name
   */
  std::string generateUniqueNodeName();

  /** \brief Initialize TF listener used for storing transforms */
  void activateListener();

  /** \brief Deactivate TF listener */
  void deactivateListener();

  /** \brief Get a static transform from local TF buffer.
   * \param target_frame the frame to which data should be transformed
   * \param source_frame the frame where the data originated
   * \return an optional containing the transform if successful, or empty if not
   */
  std::optional<TransformStamped> getStaticTransform(
    const std::string & target_frame, const std::string & source_frame);

  /** \brief Get an unknown (static or dynamic) transform.
   * \param target_frame the frame to which data should be transformed
   * \param source_frame the frame where the data originated
   * \param time the time at which the value of the transform is desired (0 will get the latest)
   * \param timeout how long to block before failing
   * \return transform if successful
   */
  TransformStamped getUnknownTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout);

  /** \brief Test if a transform is possible with static or dynamic buffer.
   * \param target_frame The frame into which to transform
   * \param source_frame The frame from which to transform
   * \param target_time The time at which to transform
   * \param timeout How long to block before failing
   * \return True if the transform is possible, false otherwise
   */
  bool canUnknownTransform(
    const std::string & target_frame, const std::string & source_frame, const tf2::TimePoint & time,
    const tf2::Duration timeout, std::string * errstr = nullptr);

  /** \brief Register TF buffer as unknown. */
  void registerAsUnknown();

  /** \brief Register TF buffer as dynamic. */
  void registerAsDynamic();

  rclcpp::Node::SharedPtr node_{nullptr};
  rclcpp::Clock::SharedPtr clock_{nullptr};
  std::unique_ptr<std::thread> executor_thread_{nullptr};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<TFMap> static_tf_buffer_{nullptr};
  rclcpp::NodeOptions options_;
  std::function<TransformStamped(
      const std::string &, const std::string &, const tf2::TimePoint &, const tf2::Duration)>
  get_transform_;
  std::function<bool(
      const std::string &, const std::string &, const tf2::TimePoint &,
      const tf2::Duration, std::string *)> can_transform_;
  std::mt19937 random_engine_;
  std::uniform_int_distribution<> dis_;
};

}  // namespace tf2_ros

#endif  // TF2_ROS__MANAGED_TRANSFORM_BUFFER_HPP_
