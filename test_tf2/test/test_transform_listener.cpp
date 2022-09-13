/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Open Source Robotics Foundation, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>


TEST(transform_listener, authority_present)
{
  const std::string pub_namespace = "/tflistnertest/ns";
  const std::string pub_node_name = "sentinel_node_name";
  auto pub_node = std::make_shared<rclcpp::Node>(pub_node_name, pub_namespace);
  auto listener_node = std::make_shared<rclcpp::Node>("listener");

  auto pub = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(
    pub_node, "/tf", tf2_ros::DynamicBroadcasterQoS());

  
  auto buffer = tf2_ros::Buffer(listener_node->get_clock());
  auto listener = tf2_ros::TransformListener(buffer, listener_node);

  // Wait for pub/sub to match
  while(pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto msg = tf2_msgs::msg::TFMessage();
  auto tf_stamped = geometry_msgs::msg::TransformStamped();
  tf_stamped.header.frame_id = "parent";
  tf_stamped.child_frame_id = "child";
  tf_stamped.header.stamp = pub_node->get_clock()->now();
  msg.transforms.push_back(tf_stamped);

  pub->publish(msg);

  // Wait for sub to get message
  while(!buffer.canTransform("parent", "child", tf2::TimePointZero)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  const std::string as_yaml = buffer.allFramesAsYAML();
  EXPECT_TRUE(as_yaml.find(pub_node_name) != std::string::npos) << as_yaml;
  EXPECT_TRUE(as_yaml.find(pub_namespace) != std::string::npos) << as_yaml;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
