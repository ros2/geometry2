/*
 * Copyright (c) 2014, Open Source Robotics Foundation
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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

void spin_for_a_second(std::shared_ptr<rclcpp::Node>& node)
{
  rclcpp::Rate r(10);
  rclcpp::spin_some(node);
  for (int i = 0; i < 10; ++i)
  {
    r.sleep();
    rclcpp::spin_some(node);
  }
}

TEST(tf2_ros_time_reset_test, time_backwards)
{
  std::shared_ptr<rclcpp::Node> node_ = std::make_shared<rclcpp::Node>("transform_listener_backwards_reset");
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  tf2_ros::Buffer buffer(clock);
  tf2_ros::TransformListener tfl(buffer);
  tf2_ros::TransformBroadcaster tfb(node_);

  auto clock_pub = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);


  rosgraph_msgs::msg::Clock c;
  c.clock = rclcpp::Time(100, 0);
  clock_pub->publish(c);

  // basic test
  ASSERT_FALSE(buffer.canTransform("foo", "bar", rclcpp::Time(101, 0)));

  // set the transform
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = rclcpp::Time(100, 0);
  msg.header.frame_id = "foo";
  msg.child_frame_id = "bar";
  msg.transform.rotation.w = 0.0;
  msg.transform.rotation.x = 1.0;
  msg.transform.rotation.y = 0.0;
  msg.transform.rotation.z = 0.0;
  tfb.sendTransform(msg);
  msg.header.stamp = rclcpp::Time(102, 0);
  tfb.sendTransform(msg);

  // make sure it arrives
  spin_for_a_second(node_);

  // verify it's been set
  ASSERT_TRUE(buffer.canTransform("foo", "bar", rclcpp::Time(101, 0)));

  // TODO (ahcorde). review this
  // c.clock.sec = 90;
  // c.clock.nanosec = 0;
  // clock_pub->publish(c);
  //
  // // make sure it arrives
  // rclcpp::spin_some(node_);
  // sleep(1);
  //
  // //Send anoterh message to trigger clock test on an unrelated frame
  // msg.header.stamp.sec = 110;
  // msg.header.stamp.nanosec = 0;
  // msg.header.frame_id = "foo2";
  // msg.child_frame_id = "bar2";
  // tfb.sendTransform(msg);
  //
  // // make sure it arrives
  // rclcpp::spin_some(node_);
  // sleep(1);
  //
  // //verify the data's been cleared
  // ASSERT_FALSE(buffer.canTransform("foo", "bar", tf2::timeFromSec(101)));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
