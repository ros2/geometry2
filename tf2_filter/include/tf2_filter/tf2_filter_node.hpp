// Copyright (c) 2018 Robert Bosch GmbH
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


#ifndef TF2_FILTER__TF2_FILTER_NODE_HPP_
#define TF2_FILTER__TF2_FILTER_NODE_HPP_

#include <string>
#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include "tf2_filter/tf2_filter.hpp"
#include "tf2_filter/visibility_control.h"

namespace tf2_filter
{
class TF2FilterNode : public rclcpp::Node
{
public:
  explicit TF2FilterNode(
    const std::string & name, const std::string & ns = "",
    bool use_intraprocess_comms = false,
    const std::vector<rclcpp::Parameter> & initial_params = {});
  virtual ~TF2FilterNode() = default;

protected:
  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg);

private:
  const rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_in_;
  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_filter_out_;
  tf2_filter::TF2Filter filter_;
};
}  // namespace tf2_filter

#endif  // TF2_FILTER__TF2_FILTER_NODE_HPP_
