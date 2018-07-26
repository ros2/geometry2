/*
 * Copyright (c) 2018, Open Source Robotics Foundation, Inc.
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

#include <exception>
#include <chrono>
#include <gtest/gtest.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>


using namespace tf2;

TEST(test_buffer, construct_with_null_clock)
{
  EXPECT_THROW(tf2_ros::Buffer(nullptr), std::invalid_argument);
}

TEST(test_buffer, can_transform_valid_transform)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(rclcpp_time);
  transform.child_frame_id = "bar";
  transform.transform.translation.x = 42.0;
  transform.transform.translation.y = -3.14;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));

  auto output = buffer.lookupTransform("foo", "bar", tf2_time);
  EXPECT_STREQ(transform.child_frame_id.c_str(), output.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, output.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, output.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, output.transform.translation.z);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

