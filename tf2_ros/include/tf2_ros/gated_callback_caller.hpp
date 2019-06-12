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

#ifndef TF2_ROS__GATED_CALLBACK_CALLER_HPP_
#define TF2_ROS__GATED_CALLBACK_CALLER_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/async_wait_for_transform.hpp>
#include <tf2_ros/visibility_control.h>

#include <functional>
#include <mutex>
#include <vector>

namespace tf2_ros
{
/// \brief Helper to call callback only after all transforms have been received.
class GatedCallbackCaller
{
public:
  using GatedCallback = std::function<void (const std::vector<geometry_msgs::msg::TransformStamped> &)>;

  /// \brief Create a helper that calls callback once a number of transforms have been received
  /// \param[in] expected_transforms the number of transforms to wait for
  /// \param[in] callback the callback to call once all transforms are ready
  TF2_ROS_PUBLIC
  GatedCallbackCaller(size_t expected_transforms, GatedCallback callback);

  TF2_ROS_PUBLIC
  virtual ~GatedCallbackCaller() = default;

  /// \brief Get a callback for a specific call to `async_wait_for_transform()`
  /// \param[in] idx index in vector passed to gated callback of transform received by this callback
  TF2_ROS_PUBLIC
  TransformCallback
  callback_at(size_t idx);

private:
  TF2_ROS_LOCAL
  void
  operator()(size_t idx, const geometry_msgs::msg::TransformStamped & transform);

  std::mutex mtx_;

  size_t callbacks_rx_ = 0;
  const size_t expected_transforms_;
  const GatedCallback callback_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};
}  // namespace tf2_ros

#endif  // TF2_ROS__GATED_CALLBACK_CALLER_HPP_
