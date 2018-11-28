#include <cstdio>
#include <map>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using std::placeholders::_1;

namespace {
  typedef std::unordered_set<std::string> TargetSet;
  typedef std::map<std::string, TargetSet> FilterMap;

  class TF2Filter
  {
  public:
    TF2Filter(const FilterMap& relevant_transforms)
    : relevant_transforms_(relevant_transforms)
    {
    }

    /**
    * Return those transforms from the input that are relevant. If none are
    * relevant, returns an empty pointer.
    */
    tf2_msgs::msg::TFMessage::SharedPtr filter(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
      tf2_msgs::msg::TFMessage::SharedPtr filtered;
      for(geometry_msgs::msg::TransformStamped t : msg->transforms)
      {
        // first we check the origin for relevance
        const FilterMap::const_iterator it =
          relevant_transforms_.find(t.header.frame_id);
        if(it == relevant_transforms_.end())
        {
          continue;
        }
        // if found, we check the target for relevance
        if(it->second.count(t.child_frame_id) > 0)
        {
          if(!filtered)
            filtered = std::make_shared<tf2_msgs::msg::TFMessage>();

          filtered->transforms.push_back(t);
        }
      }

      return filtered;
    }

  private:
    FilterMap relevant_transforms_;
  };

  class TF2FilterNode : public rclcpp::Node
  {
  public:
    TF2FilterNode()
    : Node("tf2_filter"), tf_in_(this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", std::bind(&TF2FilterNode::tf_cb, this, _1))),
      tf_filter_out_(this->create_publisher<tf2_msgs::msg::TFMessage>(
          "/tf_filtered"))
    {
    }

  protected:
    void tf_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
      tf2_msgs::msg::TFMessage::SharedPtr filtered(filter_->filter(msg));
      if(filtered)
      {
        tf_filter_out_->publish(filtered);
      }
    }

  private:
    const rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_in_;
    const rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_filter_out_;
    TF2Filter* filter_;
  };
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TF2FilterNode>());

  rclcpp::shutdown();

  return 0;
}
