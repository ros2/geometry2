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

#include <gtest/gtest.h>
#include <memory>
#include "tf2_filter/tf2_filter.hpp"

using tf2_filter::TF2Filter;
using tf2_filter::FilterMap;
using tf2_filter::TargetSet;
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
}  // namespace


TEST_F(TF2FilterTest, filter_empty)
{
  FilterMap map;
  TF2Filter filter(map);

  ASSERT_FALSE(filter.filter(msg));
}

TEST_F(TF2FilterTest, filter_empty_set)
{
  FilterMap map;
  map["base_link"] = TargetSet();
  TF2Filter filter(map);

  ASSERT_FALSE(filter.filter(msg));
}


TEST_F(TF2FilterTest, filter_single)
{
  TargetSet ts;
  ts.emplace("arm1");
  FilterMap map;
  map["base_link"] = ts;

  TF2Filter filter = TF2Filter(map);
  ASSERT_TRUE(filter.filter(msg));

  msg->transforms[0].child_frame_id = "arm2";
  ASSERT_FALSE(filter.filter(msg));

  msg->transforms[0].child_frame_id = "arm1a";
  ASSERT_FALSE(filter.filter(msg));

}

TEST_F(TF2FilterTest, filter_double)
{
  TargetSet ts;
  ts.emplace("arm1");
  ts.emplace("leg1");
  FilterMap map;
  map["base_link"] = ts;

  TF2Filter filter = TF2Filter(map);
  ASSERT_TRUE(filter.filter(msg));

  msg->transforms[0].child_frame_id = "arm2";
  ASSERT_FALSE(filter.filter(msg));

  msg->transforms[0].child_frame_id = "leg1";
  ASSERT_TRUE(filter.filter(msg));
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
