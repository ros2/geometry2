// Copyright 2025 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc.

#include <tf2_ros/create_timer_ros.h>
#include "tf2_ros/managed_transform_buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>

class TestManagedTransformBuffer : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{nullptr};
  geometry_msgs::msg::TransformStamped tf_root_to_child_1_;
  geometry_msgs::msg::TransformStamped tf_root_to_child_2_;
  geometry_msgs::msg::TransformStamped tf_child_2_to_child_3_;
  geometry_msgs::msg::TransformStamped tf_child_3_to_child_4_;
  tf2::Transform tf2_root_to_child_1_;
  tf2::Transform tf2_root_to_child_2_;
  tf2::Transform tf2_child_2_to_child_3_;
  tf2::Transform tf2_child_3_to_child_4_;
  double precision_;

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const int32_t seconds, const uint32_t nanoseconds, const std::string & parent_frame,
    const std::string & child_frame, double x, double y, double z, double qx, double qy, double qz,
    double qw)
  {
    rclcpp::Time timestamp(seconds, nanoseconds, node_->get_clock()->get_clock_type());
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  tf2::Transform msgToTf2(const geometry_msgs::msg::Transform & msg) const
  {
    auto tf = tf2::Transform();
    tf.setOrigin(tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
    tf.setRotation(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w));
    return tf;
  }

  geometry_msgs::msg::Transform tf2ToMsg(const tf2::Transform & tf2) const
  {
    geometry_msgs::msg::Transform out;
    out.translation.x = tf2.getOrigin().getX();
    out.translation.y = tf2.getOrigin().getY();
    out.translation.z = tf2.getOrigin().getZ();
    out.rotation.w = tf2.getRotation().getW();
    out.rotation.x = tf2.getRotation().getX();
    out.rotation.y = tf2.getRotation().getY();
    out.rotation.z = tf2.getRotation().getZ();
    return out;
  }

  void broadcastDynamicTf(geometry_msgs::msg::TransformStamped transform, uint32_t seconds = 1)
  {
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), [this, transform]() -> void {
        tf_broadcaster_->sendTransform(transform);
      });

    rclcpp::Rate r(10);
    rclcpp::spin_some(node_);
    for (uint32_t i = 0; i < 10u * seconds; ++i) {
      r.sleep();
      rclcpp::spin_some(node_);
    }

    timer_->cancel();
    timer_->reset();
  }

  void expectTf2Equal(const tf2::Transform & tf1, const tf2::Transform & tf2) const
  {
    EXPECT_NEAR(tf1.getOrigin().getX(), tf2.getOrigin().getX(), precision_);
    EXPECT_NEAR(tf1.getOrigin().getY(), tf2.getOrigin().getY(), precision_);
    EXPECT_NEAR(tf1.getOrigin().getZ(), tf2.getOrigin().getZ(), precision_);
    EXPECT_NEAR(tf1.getRotation().getX(), tf2.getRotation().getX(), precision_);
    EXPECT_NEAR(tf1.getRotation().getY(), tf2.getRotation().getY(), precision_);
    EXPECT_NEAR(tf1.getRotation().getZ(), tf2.getRotation().getZ(), precision_);
    EXPECT_NEAR(tf1.getRotation().getW(), tf2.getRotation().getW(), precision_);
  }

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time = rclcpp::Time(100, 0),
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(1.0))
  {
    try {
      auto tf = managed_tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
      return std::make_optional<geometry_msgs::msg::TransformStamped>(tf);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Failure to get transform from %s to %s with error: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time = rclcpp::Time(100, 0),
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(1.0))
  {
    std::string error;
    return managed_tf_buffer_->canTransform(target_frame, source_frame, time, timeout, &error);
  }

  void SetUp() override
  {
    node_ = std::make_unique<rclcpp::Node>("test_managed_transform_buffer");

    managed_tf_buffer_ =
      std::make_unique<tf2_ros::ManagedTransformBuffer>(
      node_->get_clock(),
      tf2::Duration(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME));
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    /*          frame_root
     *          /        \
     * frame_child_1   frame_child_2
     *                      |(dynamic)
     *                 frame_child_3
     *                      |
     *                 frame_child_4
     */
    tf_root_to_child_1_ = generateTransformMsg(
      100, 0, "frame_root", "frame_child_1", 0.690, 0.000, 2.100, -0.007, -0.007, 0.692, 0.722);
    tf_root_to_child_2_ = generateTransformMsg(
      100, 0, "frame_root", "frame_child_2", 0.0, -0.56362, -0.30555, 0.244, 0.248, 0.665, 0.661);
    tf_child_2_to_child_3_ = generateTransformMsg(
      100, 0, "frame_child_2", "frame_child_3", 1.0, 0.35, 0.0, 0.0, 0.0, 0.0, 1.0);
    tf_child_3_to_child_4_ = generateTransformMsg(
      100, 0, "frame_child_3", "frame_child_4", 0.0, 0.125, 0.5, 0.0, 0.0, 0.0, 1.0);
    tf2_root_to_child_1_ = msgToTf2(tf_root_to_child_1_.transform);
    tf2_root_to_child_2_ = msgToTf2(tf_root_to_child_2_.transform);
    tf2_child_2_to_child_3_ = msgToTf2(tf_child_2_to_child_3_.transform);
    tf2_child_3_to_child_4_ = msgToTf2(tf_child_3_to_child_4_.transform);
    precision_ = 0.01;

    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown() override
  {
  }
};

TEST_F(TestManagedTransformBuffer, TestReturn)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);

  auto tf_root_to_child_1 = getTransform("frame_root", "frame_child_1");
  EXPECT_TRUE(tf_root_to_child_1.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNoExist)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);

  auto tf_root_to_fake = getTransform("frame_root", "frame_fake");
  EXPECT_FALSE(tf_root_to_fake.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformSameFrame)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);

  auto tf_root_to_root = getTransform("frame_root", "frame_root");
  EXPECT_TRUE(tf_root_to_root.has_value());
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformInverse)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);

  auto tf_child_1_to_root = getTransform("frame_child_1", "frame_root");
  ASSERT_TRUE(tf_child_1_to_root.has_value());
  auto tf2_child_1_to_root_inv = msgToTf2(tf_child_1_to_root.value().transform).inverse();
  expectTf2Equal(tf2_child_1_to_root_inv, tf2_root_to_child_1_);
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformNonDirect)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);
  static_tf_broadcaster_->sendTransform(tf_root_to_child_2_);

  auto tf_child_1_to_child_2 = getTransform("frame_child_1", "frame_child_2");
  ASSERT_TRUE(tf_child_1_to_child_2.has_value());
  auto tf2_child_1_to_child_2 = msgToTf2(tf_child_1_to_child_2.value().transform);
  expectTf2Equal(tf2_child_1_to_child_2, tf2_root_to_child_1_.inverse() * tf2_root_to_child_2_);
  EXPECT_TRUE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformDynamic)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);
  static_tf_broadcaster_->sendTransform(tf_root_to_child_2_);

  std::future<void> future =
    std::async(std::launch::async, [this]() {broadcastDynamicTf(tf_child_2_to_child_3_);});
  auto tf_child_2_to_child_3 = getTransform("frame_child_2", "frame_child_3", rclcpp::Time(100, 0));
  future.wait();

  ASSERT_TRUE(tf_child_2_to_child_3.has_value());
  auto tf2_child_2_to_child_3 = msgToTf2(tf_child_2_to_child_3.value().transform);
  expectTf2Equal(tf2_child_2_to_child_3, tf2_child_2_to_child_3_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  auto tf_root_to_child_3 = getTransform("frame_root", "frame_child_3");
  ASSERT_TRUE(tf_root_to_child_3.has_value());
  auto tf2_root_to_child_3 = msgToTf2(tf_root_to_child_3.value().transform);
  expectTf2Equal(tf2_root_to_child_3, tf2_root_to_child_2_ * tf2_child_2_to_child_3_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  auto tf_child_3_to_child_1 = getTransform("frame_child_3", "frame_child_1");
  ASSERT_TRUE(tf_child_3_to_child_1.has_value());
  auto tf2_child_3_to_child_1 = msgToTf2(tf_child_3_to_child_1.value().transform);
  expectTf2Equal(
    tf2_child_3_to_child_1,
    tf2_child_2_to_child_3_.inverse() * tf2_root_to_child_2_.inverse() * tf2_root_to_child_1_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestTransformMultipleCall)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);
  static_tf_broadcaster_->sendTransform(tf_root_to_child_2_);
  static_tf_broadcaster_->sendTransform(tf_child_3_to_child_4_);

  std::optional<geometry_msgs::msg::TransformStamped> tf;
  tf2::Transform tf2;
  tf = getTransform("frame_root", "frame_fake");
  EXPECT_FALSE(tf.has_value());

  tf = getTransform("frame_child_1", "frame_root");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(tf2, tf2_root_to_child_1_.inverse());

  tf = getTransform("frame_fake", "frame_fake");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(tf2, tf2::Transform::getIdentity());

  tf = getTransform("frame_child_1", "frame_child_2");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(tf2, tf2_root_to_child_1_.inverse() * tf2_root_to_child_2_);

  tf = getTransform("frame_child_4", "frame_child_3");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(tf2, tf2_child_3_to_child_4_.inverse());

  EXPECT_TRUE(managed_tf_buffer_->isStatic());

  std::future<void> future =
    std::async(std::launch::async, [this]() {broadcastDynamicTf(tf_child_2_to_child_3_);});
  tf = getTransform("frame_child_1", "frame_child_3");
  future.wait();

  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(
    tf2, tf2_root_to_child_1_.inverse() * tf2_root_to_child_2_ * tf2_child_2_to_child_3_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  tf = getTransform("frame_child_2", "frame_child_1");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(tf2, tf2_root_to_child_2_.inverse() * tf2_root_to_child_1_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  tf = getTransform("frame_child_4", "frame_child_1");
  ASSERT_TRUE(tf.has_value());
  tf2 = msgToTf2(tf.value().transform);
  expectTf2Equal(
    tf2, tf2_child_3_to_child_4_.inverse() * tf2_child_2_to_child_3_.inverse() *
    tf2_root_to_child_2_.inverse() * tf2_root_to_child_1_);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

TEST_F(TestManagedTransformBuffer, TestCanTransformMultipleCall)
{
  static_tf_broadcaster_->sendTransform(tf_root_to_child_1_);
  static_tf_broadcaster_->sendTransform(tf_root_to_child_2_);
  static_tf_broadcaster_->sendTransform(tf_child_3_to_child_4_);

  bool can_transform{false};
  can_transform = canTransform("frame_root", "frame_fake");
  EXPECT_FALSE(can_transform);

  can_transform = canTransform("frame_child_1", "frame_root");
  ASSERT_TRUE(can_transform);

  can_transform = canTransform("frame_fake", "frame_fake");
  ASSERT_TRUE(can_transform);

  can_transform = canTransform("frame_child_1", "frame_child_2");
  ASSERT_TRUE(can_transform);

  can_transform = canTransform("frame_child_4", "frame_child_3");
  ASSERT_TRUE(can_transform);

  EXPECT_TRUE(managed_tf_buffer_->isStatic());

  std::future<void> future =
    std::async(std::launch::async, [this]() {broadcastDynamicTf(tf_child_2_to_child_3_);});
  can_transform = canTransform("frame_child_1", "frame_child_3");
  future.wait();

  ASSERT_TRUE(can_transform);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  can_transform = canTransform("frame_child_2", "frame_child_1");
  ASSERT_TRUE(can_transform);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());

  can_transform = canTransform("frame_child_4", "frame_child_1");
  ASSERT_TRUE(can_transform);
  EXPECT_FALSE(managed_tf_buffer_->isStatic());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
