/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "node_wrapper.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"

using namespace std::chrono_literals;

template<typename PredicateT>
bool wait_for(PredicateT predicate, std::chrono::seconds timeout = 5s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}

void expect_internal_node_groups_are_spun(bool static_only)
{
  auto probe = rclcpp::Node::make_shared(
    static_only ? "static_listener_callback_group_probe" : "listener_callback_group_probe");
  const auto nodes_before = probe->get_node_names();
  auto parameter_events = probe->create_publisher<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", rclcpp::ParameterEventsQoS());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener listener(buffer, true, static_only);

  std::string listener_node_name;
  const auto listener_was_discovered = [&]() {
      for (const auto & node_name : probe->get_node_names()) {
        const bool is_listener =
          node_name.find("transform_listener_impl_") != std::string::npos;
        const bool is_new =
          std::find(nodes_before.begin(), nodes_before.end(), node_name) == nodes_before.end();
        if (is_listener && is_new) {
          listener_node_name = node_name;
          return true;
        }
      }
      return false;
    };
  ASSERT_TRUE(wait_for(listener_was_discovered)) <<
    "The TransformListener's internal node was not discovered";

  const auto initial_clock_subscriptions = probe->count_subscribers("/clock");
  rcl_interfaces::msg::ParameterEvent event;
  event.stamp = probe->now();
  event.node = listener_node_name;
  rcl_interfaces::msg::Parameter use_sim_time;
  use_sim_time.name = "use_sim_time";
  use_sim_time.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  use_sim_time.value.bool_value = true;
  event.changed_parameters.push_back(use_sim_time);

  const auto default_group_was_spun = [&]() {
      parameter_events->publish(event);
      return probe->count_subscribers("/clock") > initial_clock_subscriptions;
    };
  ASSERT_TRUE(wait_for(default_group_was_spun)) <<
    "The internal node's default callback group was not spun";

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = probe->now();
  transform.header.frame_id = static_only ? "static_parent" : "dynamic_parent";
  transform.child_frame_id = static_only ? "static_child" : "dynamic_child";
  transform.transform.rotation.w = 1.0;

  if (static_only) {
    tf2_ros::StaticTransformBroadcaster broadcaster(probe);
    const auto static_transform_was_received = [&]() {
        broadcaster.sendTransform(transform);
        return buffer.canTransform(
        transform.header.frame_id, transform.child_frame_id, tf2::TimePointZero);
      };
    EXPECT_TRUE(wait_for(static_transform_was_received)) <<
      "The manually added /tf_static callback group was not spun";
  } else {
    tf2_ros::TransformBroadcaster broadcaster(probe);
    const auto transform_was_received = [&]() {
        broadcaster.sendTransform(transform);
        return buffer.canTransform(
        transform.header.frame_id, transform.child_frame_id, tf2::TimePointZero);
      };
    EXPECT_TRUE(wait_for(transform_was_received)) <<
      "The manually added /tf callback group was not spun";
  }
}

class CustomNode : public rclcpp::Node
{
public:
  CustomNode()
  : rclcpp::Node("tf2_ros_test_transform_listener_node")
  {}

  void init_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer, shared_from_this(), false);
  }

  void init_static_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf_listener_ =
      std::make_shared<tf2_ros::StaticTransformListener>(buffer, shared_from_this(), false);
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class CustomComposableNode : public rclcpp::Node
{
public:
  explicit CustomComposableNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tf2_ros_test_transform_listener_composable_node", options)
  {}

  void init_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer, shared_from_this(), false);
  }

  void init_static_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    tf_listener_ =
      std::make_shared<tf2_ros::StaticTransformListener>(buffer, shared_from_this(), false);
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

TEST(tf2_test_transform_listener, transform_listener_rclcpp_node)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, false);
}

TEST(tf2_test_transform_listener, internal_node_callback_groups)
{
  expect_internal_node_groups_are_spun(false);
}

TEST(tf2_test_transform_listener, transform_listener_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_message_filter");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, false);
}

TEST(tf2_test_transform_listener, transform_listener_as_member)
{
  auto custom_node = std::make_shared<CustomNode>();
  custom_node->init_tf_listener();
}

TEST(tf2_test_transform_listener, transform_listener_with_intraprocess)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options = options.use_intra_process_comms(true);
  auto custom_node = std::make_shared<CustomComposableNode>(options);
  custom_node->init_tf_listener();
}

TEST(tf2_test_static_transform_listener, static_transform_listener_rclcpp_node)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_static_transform_listener");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::StaticTransformListener stfl(buffer, node, false);
}

TEST(tf2_test_static_transform_listener, internal_node_callback_groups)
{
  expect_internal_node_groups_are_spun(true);
}

TEST(tf2_test_static_transform_listener, static_transform_listener_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_static_transform_listener");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::StaticTransformListener stfl(buffer, node, false);
}

TEST(tf2_test_static_transform_listener, static_transform_listener_as_member)
{
  auto custom_node = std::make_shared<CustomNode>();
  custom_node->init_static_tf_listener();
}

TEST(tf2_test_static_transform_listener, static_transform_listener_with_intraprocess)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options = options.use_intra_process_comms(true);
  auto custom_node = std::make_shared<CustomComposableNode>(options);
  custom_node->init_static_tf_listener();
}

TEST(tf2_test_listeners, static_vs_dynamic)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_static_transform_listener");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer dynamic_buffer(clock);
  tf2_ros::Buffer static_buffer(clock);
  tf2_ros::TransformListener tfl(dynamic_buffer, node, true);
  tf2_ros::StaticTransformListener stfl(static_buffer, node, true);
  tf2_ros::TransformBroadcaster broadcaster(node);
  tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

  geometry_msgs::msg::TransformStamped static_trans;
  static_trans.header.stamp = clock->now();
  static_trans.header.frame_id = "parent_static";
  static_trans.child_frame_id = "child_static";
  static_trans.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(static_trans);

  geometry_msgs::msg::TransformStamped dynamic_trans;
  dynamic_trans.header.frame_id = "parent_dynamic";
  dynamic_trans.child_frame_id = "child_dynamic";
  dynamic_trans.transform.rotation.w = 1.0;

  for (int i = 0; i < 10; ++i) {
    dynamic_trans.header.stamp = clock->now();
    broadcaster.sendTransform(dynamic_trans);

    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  // Dynamic buffer should have both dynamic and static transforms available
  EXPECT_NO_THROW(
    dynamic_buffer.lookupTransform("parent_dynamic", "child_dynamic", tf2::TimePointZero));
  EXPECT_NO_THROW(
    dynamic_buffer.lookupTransform("parent_static", "child_static", tf2::TimePointZero));

  // Static buffer should have only static transforms available
  EXPECT_THROW(
    static_buffer.lookupTransform("parent_dynamic", "child_dynamic", tf2::TimePointZero),
    tf2::LookupException);
  EXPECT_NO_THROW(
    static_buffer.lookupTransform("parent_static", "child_static", tf2::TimePointZero));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
