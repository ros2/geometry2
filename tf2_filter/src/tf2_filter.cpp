// Copyright 2018 Robert Bosch GmbH
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

#include <memory>
#include "tf2_filter/tf2_filter.hpp"

namespace tf2_filter
{

tf2_msgs::msg::TFMessage::SharedPtr TF2Filter::filter(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  tf2_msgs::msg::TFMessage::SharedPtr filtered;
  for (geometry_msgs::msg::TransformStamped t : msg->transforms) {
    // first we check the origin for relevance
    const FilterMap::const_iterator it =
      relevant_transforms_.find(t.header.frame_id);
    if (it == relevant_transforms_.end()) {
      continue;
    }
    // if found, we check the target for relevance
    if (it->second.count(t.child_frame_id) > 0) {
      if (!filtered) {
        filtered = std::make_shared<tf2_msgs::msg::TFMessage>();
      }

      filtered->transforms.push_back(t);
    }
  }

  return filtered;
}

}  // namespace tf2_filter
