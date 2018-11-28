#ifndef TF2_FILTER__TF2_FILTER_HPP_
#define TF2_FILTER__TF2_FILTER_HPP_

#include <map>
#include <unordered_set>
#include <tf2_msgs/msg/tf_message.hpp>
#include "tf2_filter/visibility_control.h"

namespace tf2_filter
{

  typedef std::unordered_set<std::string> TargetSet;
  typedef std::map<std::string, TargetSet> FilterMap;

  class TF2Filter
  {
  public:
    TF2Filter(const FilterMap& relevant_transforms)
    : relevant_transforms_(relevant_transforms)
    {
    }
    virtual ~TF2Filter()
    {
    }

    /**
    * Return those transforms from the input that are relevant. If none are
    * relevant, returns an empty pointer.
    */
    tf2_msgs::msg::TFMessage::SharedPtr filter(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  private:
    FilterMap relevant_transforms_;
  };
}  // namespace tf2_filter

#endif  // TF2_FILTER__TF2_FILTER_HPP_
