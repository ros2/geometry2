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


#ifndef TF2_FILTER__TF2_FILTER_HPP_
#define TF2_FILTER__TF2_FILTER_HPP_

#include <map>
#include <unordered_set>
#include <string>
#include "tf2_filter/visibility_control.h"
#include <tf2_msgs/msg/tf_message.hpp>

namespace tf2_filter
{
typedef std::unordered_set<std::string> TargetSet;
typedef std::map<std::string, TargetSet> FilterMap;

class TF2Filter
{
public:
  explicit TF2Filter(const FilterMap & relevant_transforms)
  : relevant_transforms_(relevant_transforms)
  {
  }
  virtual ~TF2Filter()
  {
  }

  /**
  * Return those transforms from the input that are relevant. If none are
  * relevant, returns an empty pointer.
  * TODO (lui3si): Check whether using UniquePtr is more appropriate.
  */
  tf2_msgs::msg::TFMessage::SharedPtr filter(const tf2_msgs::msg::TFMessage::SharedPtr msg);

private:
  FilterMap relevant_transforms_;
};
}  // namespace tf2_filter

#endif  // TF2_FILTER__TF2_FILTER_HPP_
