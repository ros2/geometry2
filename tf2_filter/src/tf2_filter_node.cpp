#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <tf2_filter/tf2_filter.hpp>

using std::placeholders::_1;

namespace
{
class TF2FilterNode : public rclcpp::Node
{
public:
  TF2FilterNode()
  : Node("tf2_filter"), tf_in_(this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", std::bind(&TF2FilterNode::tf_cb, this, _1))),
    tf_filter_out_(this->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_filtered")),
    filter_(nullptr)
  {

  }

protected:
  void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if (filter_) {
      tf2_msgs::msg::TFMessage::SharedPtr filtered(filter_->filter(msg));
      if (filtered) {
        tf_filter_out_->publish(filtered);
      }
    }
  }

private:
  const rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_in_;
  const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_filter_out_;
  tf2_filter::TF2Filter * filter_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TF2FilterNode>());

  rclcpp::shutdown();

  return 0;
}
