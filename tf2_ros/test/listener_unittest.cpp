/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <chrono>
#include <gtest/gtest.h>
#include <functional>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2_ros/transform_listener.h>

TEST(tf2_ros_test_listener, transform_listener)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_test_listener_transform_listener");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer, node, false);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // Start spinning in a thread
  std::thread spin_thread = std::thread(
    std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

  geometry_msgs::msg::TransformStamped ts;
  ts.transform.rotation.w = 1;
  ts.header.frame_id = "a";
  ts.header.stamp = rclcpp::Time(10, 0);
  ts.child_frame_id = "b";
  ts.transform.translation.x = 1;
  ts.transform.translation.y = 2;
  ts.transform.translation.z = 3;

  buffer.setTransform(ts, "authority");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_TRUE(buffer.canTransform("a", "b", tf2::timeFromSec(0)));

  geometry_msgs::msg::TransformStamped out_rootc = buffer.lookupTransform("a", "b", builtin_interfaces::msg::Time());

  EXPECT_EQ(1, out_rootc.transform.translation.x);
  EXPECT_EQ(2, out_rootc.transform.translation.y);
  EXPECT_EQ(3, out_rootc.transform.translation.z);
  EXPECT_EQ(1, out_rootc.transform.rotation.w);
  EXPECT_EQ(0, out_rootc.transform.rotation.x);
  EXPECT_EQ(0, out_rootc.transform.rotation.y);
  EXPECT_EQ(0, out_rootc.transform.rotation.z);

  executor.cancel();
  spin_thread.join();
  node.reset();
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
