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
