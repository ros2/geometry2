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

#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_filter/tf2_filter.hpp>
#include <tf2_filter/tf2_filter_config.hpp>

using std::placeholders::_1;

namespace tf2_filter
{
class TF2FilterNode : public rclcpp::Node
{
public:
  TF2FilterNode()
  : Node("tf2_filter"), tf_in_(this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", std::bind(&TF2FilterNode::tf_cb, this, _1))),
    tf_filter_out_(this->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_filtered")),
    filter_(filtermap_from_param(this->get_parameter("frames")))
  {
  }

protected:
  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage::SharedPtr filtered(filter_.filter(msg));
    if (filtered) {
      tf_filter_out_->publish(filtered);
    }
  }

private:
  const rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_in_;
  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_filter_out_;
  tf2_filter::TF2Filter filter_;
};
}  // namespace tf2_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<tf2_filter::TF2FilterNode>());

  rclcpp::shutdown();

  return 0;
}
