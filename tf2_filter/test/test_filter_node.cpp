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
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include "tf2_filter/tf2_filter.hpp"
#include "tf2_filter/tf2_filter_config.hpp"
#include "tf2_filter/tf2_filter_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using tf2_filter::TF2Filter;
using tf2_filter::TF2FilterNode;
using tf2_filter::FilterMap;
using tf2_filter::TargetSet;
using tf2_filter::filtermap_from_frames;
using tf2_filter::filtermap_from_param;
using tf2_msgs::msg::TFMessage;
using geometry_msgs::msg::TransformStamped;
using std::chrono::milliseconds;

TFMessage::SharedPtr msg;

TF2FilterNode::SharedPtr node;
rclcpp::Subscription<TFMessage>::SharedPtr sub;
rclcpp::Publisher<TFMessage>::SharedPtr pub;
using std::placeholders::_1;
typedef std::recursive_timed_mutex MutexT;


namespace
{
class TF2FilterNodeTest : public ::testing::Test
{
public:
  TF2FilterNodeTest()
  : ready(false)
  {
    // setting up the node takes time until all connections are made,
    // so do this only once
    rclcpp::ParameterValue frames({std::string("base_link"),
        std::string("arm1")});
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("frames", frames)
    };
    // note that we configure the node for intra-process comms
    node = std::make_shared<TF2FilterNode>("tf2_filter", "", true,
        params);
    sub = node->create_subscription<TFMessage>("/tf_filtered",
        std::bind(&TF2FilterNodeTest::tf_cb, this, _1));
    pub = node->create_publisher<TFMessage>("/tf");

    // give it some time, then check that we are operational
    rclcpp::spin_some(node);
    size_t check_pub = 0, check_sub = 0;
    unsigned int count = 0;
    do {
      rclcpp::spin_some(node);
      // goes to 1 once our publisher is subscribed to
      check_pub = node->count_subscribers("/tf");
      // goes to 1 once our subscriber is connected
      check_sub = node->count_publishers("/tf_filtered");
      if (check_pub < 1 || check_sub < 1) {
        RCLCPP_INFO(node->get_logger(), "Waiting for connections to be setup");
        rclcpp::sleep_for(milliseconds(100));
      }
      ++count;
    } while ((check_pub < 1 || check_sub < 1) && count < 10);
    if(count > 10) {
      throw std::runtime_error("Connection setup not completed in time");
    }
    RCLCPP_INFO(node->get_logger(), "Node setup finished");
  }
  ~TF2FilterNodeTest()
  {
    pub.reset();
    sub.reset();
    node.reset();
  }

  template<typename Rep, typename Period>
  TFMessage::SharedPtr get_new(
    const std::chrono::duration<Rep, Period> & timeout = milliseconds(100))
  {
    std::unique_lock<MutexT> lck(mtx_);
    if (ready) {
      return msg_;
    } else {
      const unsigned int tries = 10;
      for (unsigned int i = 0; i < tries; ++i) {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "Waiting");
        receive_cv_.wait_for(lck, timeout);
        if(ready) {
          return msg_;
        }
      }
    }

    return nullptr;
  }
  void tf_cb(const TFMessage::SharedPtr msg)
  {
    RCLCPP_INFO(node->get_logger(), "Got message");
    std::unique_lock<MutexT> lck(mtx_, milliseconds(100));
    if (lck.owns_lock()) {
      msg_ = msg;
      ready = true;
      RCLCPP_INFO(node->get_logger(), "Notifying internally");
      receive_cv_.notify_all();
      lck.unlock();
    } else {
      RCLCPP_INFO(node->get_logger(), "Lock acquisition failed");
    }
  }

protected:
  virtual void SetUp()
  {
    ready = false;

    // create test message
    msg = std::make_shared<TFMessage>();
    msg->transforms.clear();
    TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "arm1";
    msg->transforms.push_back(tf);
  }

  bool ready;
  std::recursive_timed_mutex mtx_;
  std::condition_variable_any receive_cv_;
  TFMessage::SharedPtr msg_;
};
}  // namespace


TEST_F(TF2FilterNodeTest, filter_normal)
{
  pub->publish(msg);
  TFMessage::SharedPtr new_msg(get_new(milliseconds(100)));
  ASSERT_TRUE(new_msg);
}

/*TEST_F(TF2FilterNodeTest, filter_empty_set)
{
  FilterMap map;
  map["base_link"] = TargetSet();
  TF2Filter filter(map);

  ASSERT_FALSE(filter.filter(msg));
}

TEST_F(TF2FilterNodeTest, filter_single)
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

TEST_F(TF2FilterNodeTest, filter_double)
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

TEST_F(TF2FilterNodeTest, map_from_frames)
{
  FilterMap filters = filtermap_from_frames({"base_link", "arm1"});
  TargetSet targets = {"arm1"};
  ASSERT_TRUE(filters.find("base_link") != filters.end());
  ASSERT_EQ(filters["base_link"], targets);
}

TEST_F(TF2FilterNodeTest, map_from_param)
{
  rcl_interfaces::msg::Parameter pmsg;
  pmsg.name = "frames";
  pmsg.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  pmsg.value.string_array_value = {"base_link", "arm1"};

  FilterMap filters = filtermap_from_param(
    rclcpp::Parameter::from_parameter_msg(pmsg));
  TargetSet targets = {"arm1"};
  ASSERT_TRUE(filters.find("base_link") != filters.end());
  ASSERT_EQ(filters["base_link"], targets);
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cerr << "Done" << std::endl;
  return result;
}
