#include <gtest/gtest.h>
#include "tf2_filter/tf2_filter.hpp"

using namespace tf2_filter;
using tf2_msgs::msg::TFMessage;
using geometry_msgs::msg::TransformStamped;

TFMessage::SharedPtr msg;

namespace
{
class TF2FilterTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    // create test message
    msg = std::make_shared<TFMessage>();
    msg->transforms.clear();
    TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "arm1";
    msg->transforms.push_back(tf);
  }
};
}


TEST_F(TF2FilterTest, filter_empty)
{
  FilterMap map;
  TF2Filter filter(map);

  ASSERT_FALSE(filter.filter(msg));

  // basic test
  //ASSERT_FALSE(buffer.canTransform("foo", "bar", builtin_interfaces::msg::Time(101, 0)));

  // verify it's been set
  //ASSERT_TRUE(buffer.canTransform("foo", "bar", builtin_interfaces::msg::Time(101, 0)));

  //verify the data's been cleared
  //ASSERT_FALSE(buffer.canTransform("foo", "bar", builtin_interfaces::msg::Time(101, 0)));

}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
