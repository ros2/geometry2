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


#ifndef TF2_FILTER__TF2_FILTER_CONFIG_HPP_
#define TF2_FILTER__TF2_FILTER_CONFIG_HPP_

#include <string>
#include <vector>
#include <tf2_filter/tf2_filter.hpp>
#include <rclcpp/parameter.hpp>

namespace tf2_filter
{
/** Create a filter map from the list in param.
 * The value is expected to be a string array (see filtermap_from_frames).
*/
FilterMap filtermap_from_params(const rclcpp::Parameter & param);

/**
 * Create a filter map from the list of frames.
 * Frame ids must start with the top-level parent frame and be in
 * in the order of the TF tree, and there must be at least two frame ids.
 */
FilterMap filtermap_from_frames(const std::vector<std::string> & frame_ids);
}  // namespace tf2_filter

#endif  // TF2_FILTER__TF2_FILTER_CONFIG_HPP_
