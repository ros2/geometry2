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

#include <memory>

#include <tf2_ros/static_transform_broadcaster.hpp>

#include "node_wrapper.hpp"

class CustomNode : public rclcpp::Node
{
public:
  CustomNode()
  : rclcpp::Node("tf2_ros_test_static_transform_broadcaster_node")
  {}

  void init_tf_broadcaster()
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

class CustomComposableNode : public rclcpp::Node
{
public:
  explicit CustomComposableNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("tf2_ros_test_static_transform_broadcaster_composable_node", options)
  {}

  void init_tf_broadcaster()
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

TEST(tf2_test_static_transform_broadcaster, transform_broadcaster_rclcpp_node)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");

  // Construct static tf broadcaster from node pointer
  {
    tf2_ros::StaticTransformBroadcaster tfb(node);
  }
  // Construct static tf broadcaster from node object
  {
    tf2_ros::StaticTransformBroadcaster tfb(*node);
  }
  // Construct static tf broadcaster from node interfaces
  {
    tf2_ros::StaticTransformBroadcaster tfb(
      node->get_node_parameters_interface(),
      node->get_node_topics_interface());
  }
}

TEST(tf2_test_static_transform_broadcaster, transform_broadcaster_with_intraprocess)
{
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options = options.use_intra_process_comms(true);
  auto custom_node = std::make_shared<CustomComposableNode>(options);
  custom_node->init_tf_broadcaster();
}

TEST(tf2_test_static_transform_broadcaster, transform_broadcaster_custom_rclcpp_node)
{
  auto node = std::make_shared<NodeWrapper>("tf2_ros_message_filter");

  // Construct static tf broadcaster from node pointer
  {
    tf2_ros::StaticTransformBroadcaster tfb(node);
  }
  // Construct static tf broadcaster from node object
  {
    tf2_ros::StaticTransformBroadcaster tfb(*node);
  }
  // Construct static tf broadcaster from node interfaces
  {
    tf2_ros::StaticTransformBroadcaster tfb(
      node->get_node_parameters_interface(),
      node->get_node_topics_interface());
  }
}

TEST(tf2_test_static_transform_broadcaster, transform_broadcaster_as_member)
{
  auto custom_node = std::make_shared<CustomNode>();
  custom_node->init_tf_broadcaster();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
