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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>

#include "node_wrapper.hpp"

class CustomNode : public rclcpp::Node
{
public:
  CustomNode()
  : rclcpp::Node("tf2_ros_test_transform_listener_node")
  {}

  void init_tf_listener()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock, *this);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer, *shared_from_this(), false);
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
    tf2_ros::Buffer buffer(clock, *this);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer, *shared_from_this(), false);
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
  tf2_ros::Buffer buffer(clock, *node);
  tf2_ros::TransformListener tfl(buffer, *node, false);
}

TEST(tf2_test_transform_listener, transform_listener_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_message_filter");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock, *node);
  tf2_ros::TransformListener tfl(buffer, *node, false);
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
}

TEST(tf2_test_static_transform_listener, static_transform_listener_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_static_transform_listener");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  tf2_ros::StaticTransformListener tfl(buffer, node, false);
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
  EXPECT_NO_THROW(dynamic_buffer.lookupTransform("parent_static", "child_static", clock->now()));

  // Static buffer should have only static transforms available
  EXPECT_THROW(
    static_buffer.lookupTransform("parent_dynamic", "child_dynamic", tf2::TimePointZero),
    tf2::LookupException);
  EXPECT_NO_THROW(static_buffer.lookupTransform("parent_static", "child_static", clock->now()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
