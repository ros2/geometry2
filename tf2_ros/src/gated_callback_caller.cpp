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

#include <tf2_ros/gated_callback_caller.hpp>

#include <stdexcept>

namespace tf2_ros
{
GatedCallbackCaller::GatedCallbackCaller(size_t expected_transforms, GatedCallback callback)
  : expected_transforms_(expected_transforms), callback_(callback)
{
  transforms_.resize(expected_transforms);
}

TransformCallback
GatedCallbackCaller::callback_at(size_t idx)
{
  if (idx >= transforms_.size()) {
    throw std::out_of_range("Asked for more callbacks than instance expected.");
  }
  return std::bind(&GatedCallbackCaller::operator(), this, idx, std::placeholders::_1);
}

void
GatedCallbackCaller::operator()(size_t idx, const geometry_msgs::msg::TransformStamped & transform)
{
  std::unique_lock<std::mutex> lock(mtx_);
  ++callbacks_rx_;
  transforms_.at(idx) = transform;
  if (callbacks_rx_ == expected_transforms_) {
    callback_(transforms_);
  }
}
}  // namespace tf2_ros
