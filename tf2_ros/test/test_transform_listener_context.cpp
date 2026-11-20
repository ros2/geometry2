/*
 * Copyright (c) 2026
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
 *     * Neither the name of the copyright holder nor the names of its
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

#include <chrono>
#include <memory>
#include <thread>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"

// The default context is never initialized in this test binary, so the
// listener's dedicated thread only works if it spins in the node's context.
TEST(tf2_test_transform_listener_context, spin_thread_uses_node_context)
{
  ASSERT_FALSE(rclcpp::ok());

  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);

  {
    auto node = std::make_shared<rclcpp::Node>(
      "listener_context_test", rclcpp::NodeOptions().context(context));
    tf2_ros::Buffer buffer(node->get_clock());

    std::unique_ptr<tf2_ros::TransformListener> listener;
    ASSERT_NO_THROW(
      listener = std::make_unique<tf2_ros::TransformListener>(buffer, *node, true));

    tf2_ros::StaticTransformBroadcaster broadcaster(*node);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "parent";
    transform.child_frame_id = "child";
    transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(transform);

    // Only the listener's own thread spins, so the transform arriving shows that it runs
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!buffer.canTransform("parent", "child", tf2::TimePointZero) &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_TRUE(buffer.canTransform("parent", "child", tf2::TimePointZero));
  }

  context->shutdown("test done");
}
