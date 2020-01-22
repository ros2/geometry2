/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_bullet/tf2_bullet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_ros/buffer_client.h>

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

static const double EPS = 1e-3;

TEST(tf2_ros, buffer_client)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("tf_action_node");
  std::unique_ptr<tf2_ros::BufferClient> client = std::make_unique<tf2_ros::BufferClient>(node, "tf_action");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  // Start spinning in a thread
  std::thread spin_thread = std::thread(
    std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

  //make sure that things are set up
  ASSERT_TRUE(client->waitForServer(std::chrono::seconds(4)));

  geometry_msgs::msg::PointStamped p1;
  p1.header.frame_id = "a";
  p1.header.stamp = builtin_interfaces::msg::Time();
  p1.point.x = 0.0;
  p1.point.y = 0.0;
  p1.point.z = 0.0;

  try
  {
    geometry_msgs::msg::PointStamped p2 = client->transform(p1, "b");
    RCLCPP_INFO(node->get_logger(),
                "p1: (%.2f, %.2f, %.2f), p2: (%.2f, %.2f, %.2f)", p1.point.x,
                p1.point.y, p1.point.z, p2.point.x, p2.point.y, p2.point.z);

    EXPECT_NEAR(p2.point.x, -5.0, EPS);
    EXPECT_NEAR(p2.point.y, -6.0, EPS);
    EXPECT_NEAR(p2.point.z, -7.0, EPS);
  }
  catch(tf2::TransformException& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transform: %s", ex.what());
    ASSERT_FALSE("Should not get here");
  }
  executor.cancel();
  spin_thread.join();
  node.reset();
}

TEST(tf2_ros, buffer_client_different_types)
{
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("tf_action_node");
  std::unique_ptr<tf2_ros::BufferClient> client = std::make_unique<tf2_ros::BufferClient>(node, "tf_action");

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  // Start spinning in a thread
  std::thread spin_thread = std::thread(
    std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

  //make sure that things are set up
  ASSERT_TRUE(client->waitForServer(std::chrono::seconds(4)));

  tf2::Stamped<KDL::Vector> k1(KDL::Vector(0, 0, 0), tf2::TimePoint(), "a");

  try
  {
    tf2::Stamped<btVector3> b1;
    client->transform(k1, b1, "b");
    RCLCPP_ERROR(node->get_logger(), "Bullet: (%.4f, %.4f, %.4f)", b1[0], b1[1], b1[2]);
    RCLCPP_ERROR(node->get_logger(), "KDL: (%.4f, %.4f, %.4f)", k1[0], k1[1], k1[2]);
    EXPECT_NEAR(b1[0], -5.0, EPS);
    EXPECT_NEAR(b1[1], -6.0, EPS);
    EXPECT_NEAR(b1[2], -7.0, EPS);
    EXPECT_EQ(b1.frame_id_, "b");
    EXPECT_EQ(k1.frame_id_, "a");
  }
  catch(tf2::TransformException& ex)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to transform: %s", ex.what());
    ASSERT_FALSE("Should not get here");
  }
  executor.cancel();
  spin_thread.join();
  node.reset();

}

int main(int argc, char** argv)
{
  // This is needed because we need to wait a little bit for the other nodes
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
