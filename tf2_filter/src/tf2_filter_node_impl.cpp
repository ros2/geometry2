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

#include <string>
#include <vector>
#include "tf2_filter/tf2_filter_node.hpp"
#include "tf2_filter/tf2_filter_config.hpp"

using std::placeholders::_1;
using rclcpp::contexts::default_context::get_global_default_context;

namespace tf2_filter
{
TF2FilterNode::TF2FilterNode(
  const std::string & name, const std::string & ns,
  bool use_intraprocess_comms,
  const std::vector<rclcpp::Parameter> & initial_params)
: Node(name, ns, get_global_default_context(),
    {}, initial_params, true, use_intraprocess_comms, true),
  tf_in_(this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", std::bind(&TF2FilterNode::tf_cb, this, _1))),
  tf_filter_out_(this->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_filtered")),
  filter_(filtermap_from_param(this->get_parameter("frames")))
{
}
TF2FilterNode::~TF2FilterNode()
{
}

void TF2FilterNode::tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  tf2_msgs::msg::TFMessage::SharedPtr filtered(filter_.filter(msg));
  if (filtered) {
    tf_filter_out_->publish(filtered);
  }
}
}  // namespace tf2_filter
