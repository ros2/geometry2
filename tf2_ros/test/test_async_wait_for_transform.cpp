/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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

#include <tf2_ros/async_wait_for_transform.hpp>
#include <tf2_ros/gated_callback_caller.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>

class AsyncWaitForTransformTest : public ::testing::Test {
 protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp::Node>("test_async_wait_for_transform", opts);

    // Create buffer/listener using existing node and rclcpp executor
    buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    const bool use_tf_background_thread = false;
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_,
      node_,
      use_tf_background_thread);
    buffer_->setUsingDedicatedThread(true);

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};


TEST_F(AsyncWaitForTransformTest, get_transform_async)
{
  std::atomic<bool> got_transform{false};
  std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>> future_ptr;

  // asynchronously get the transform using the executor
  future_ptr = async_wait_for_transform(
    node_,
    buffer_,
    "foo",
    "bar",
    rclcpp::Time(123, 456),
    rclcpp::Duration(std::chrono::milliseconds(500)),
    [&got_transform] (const geometry_msgs::msg::TransformStamped &) {
      got_transform = true;
    });

  // Send the transform being waited for
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp.sec = 123;
  transform.header.stamp.nanosec = 456;
  transform.child_frame_id = "bar";
  transform.transform.rotation.w = 1.0;
  broadcaster_->sendTransform(transform);

  rclcpp::spin_until_future_complete(node_, *future_ptr);
  EXPECT_TRUE(got_transform.load());
}

TEST_F(AsyncWaitForTransformTest, get_transform_async_gated)
{
  std::atomic<bool> got_transform{false};
  std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>> first_future_ptr;
  std::shared_ptr<std::shared_future<geometry_msgs::msg::TransformStamped>> second_future_ptr;

  const size_t num_transforms = 2;
  tf2_ros::GatedCallbackCaller gcbc(
    num_transforms,
    [&got_transform](const std::vector<geometry_msgs::msg::TransformStamped> &) {
      got_transform = true;
    });

  // asynchronously get the transform using the executor
  first_future_ptr = async_wait_for_transform(
    node_,
    buffer_,
    "ping",
    "pong",
    rclcpp::Time(123, 456),
    rclcpp::Duration(std::chrono::milliseconds(500)),
    gcbc.callback_at(0));

  second_future_ptr = async_wait_for_transform(
    node_,
    buffer_,
    "marco",
    "polo",
    rclcpp::Time(123, 456),
    rclcpp::Duration(std::chrono::milliseconds(500)),
    gcbc.callback_at(1));

  // Send the transform being waited for
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "ping";
  transform.header.stamp.sec = 123;
  transform.header.stamp.nanosec = 456;
  transform.child_frame_id = "pong";
  transform.transform.rotation.w = 1.0;
  broadcaster_->sendTransform(transform);

  rclcpp::spin_until_future_complete(node_, *first_future_ptr);
  EXPECT_FALSE(got_transform.load());

  transform.header.frame_id = "marco";
  transform.child_frame_id = "polo";
  broadcaster_->sendTransform(transform);

  rclcpp::spin_until_future_complete(node_, *second_future_ptr);
  EXPECT_TRUE(got_transform.load());
}
