#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace {
  class TF2Filter : public rclcpp::Node
  {
  public:
    TF2Filter()
    : Node("tf2_filter")
    {
      tf_filter_out_ = this->create_publisher<tf2_msgs::TFMessage>(
        "/tf_filtered");

    }
  private:
    rclcpp::Subscriber tf_in_;
    rclcpp::Publisher tf_filter_out_;

  };
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  TF2Filter filter;
  
  return 0;
}
