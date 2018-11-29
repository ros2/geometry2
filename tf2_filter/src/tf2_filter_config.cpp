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
#include <string>
#include <vector>
#include "tf2_filter/tf2_filter_config.hpp"

namespace tf2_filter
{
typedef std::vector<std::string> s_array;

FilterMap filtermap_from_params(const rclcpp::Parameter & param)
{
  if (param.get_type() ==
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY)
  {
    return filtermap_from_frames(param.as_string_array());
  } else {
    throw std::invalid_argument(std::string(
              "Parameter must be string array, not ") + param.get_type_name());
  }
}

FilterMap filtermap_from_frames(const std::vector<std::string> & frame_ids)
{
  if (frame_ids.size() < 2) {
    throw std::invalid_argument("Need at least two frames");
  }
  FilterMap filters;

  s_array::const_iterator parent = frame_ids.begin();
  s_array::const_iterator child = parent + 1;
  while (child != frame_ids.end()) {
    filters[*parent].insert(*child);
    ++parent;
    ++child;
  }

  return filters;
}
}  // namespace tf2_filter
