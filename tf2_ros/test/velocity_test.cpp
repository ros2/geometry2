/*
 * Copyright (c) 2024, Open Source Robotics Foundation, Inc.
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

#include <vector>
#include <string>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_interface.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/transform_listener.hpp"

// #include "tf/LinearMath/Vector3.h"

// The fixture for testing class Foo.
class LinearVelocitySquareTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  LinearVelocitySquareTest()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock);

    rclcpp::Time rclcpp_time = clock->now();
    tf2_time_ = tf2::TimePoint(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    double x = -.1;
    double y = 0;
    double z = 0;
    for (double t = 0.1; t <= 6; t += 0.1) {
      if (t < 1) {
        x += .1;
      } else if (t < 2) {
        y += .1;
      } else if (t < 3) {
        x -= .1;
      } else if (t < 4) {
        y -= .1;
      } else if (t < 5) {
        z += .1;
      } else {
        z -= .1;
      }

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.header.frame_id = "foo";
        transform.child_frame_id = "bar";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.translation.x = 1;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;
        transform.header.frame_id = "foo";
        transform.child_frame_id = "stationary_offset_child";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 1;
        transform.header.frame_id = "stationary_offset_parent";
        transform.child_frame_id = "foo";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }
    }

    // You can do set-up work for each test here.
  }

  virtual ~LinearVelocitySquareTest()
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp()
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown()
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for LinearVelocity.
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  tf2::TimePoint tf2_time_;
};

// The fixture for testing class Foo.
class AngularVelocitySquareTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  AngularVelocitySquareTest()
  {
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    buffer_ = std::make_shared<tf2_ros::Buffer>(clock);

    rclcpp::Time rclcpp_time = clock->now();
    tf2_time_ = tf2::TimePoint(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

    double x = -.1;
    double y = 0;
    double z = 0;
    for (double t = 0.1; t < 6; t += 0.1) {
      if (t < 1) {x += .1;} else if (t < 2) {x -= .1;} else if (t < 3) {y += .1;} else if (t < 4) {
        y -= .1;
      } else if (t < 5) {z += .1;} else {z -= .1;}

      tf2::Quaternion quat;
      quat.setRPY(x, y, z);
      quat = quat.normalize();

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = quat.w();
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.header.frame_id = "foo";
        transform.child_frame_id = "bar";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.translation.x = 1.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.header.frame_id = "foo";
        transform.child_frame_id = "stationary_offset_child";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }

      {
        geometry_msgs::msg::TransformStamped transform;
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = -1.0;
        transform.header.frame_id = "stationary_offset_parent";
        transform.child_frame_id = "foo";
        transform.header.stamp =
          builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration::from_seconds(t));
        buffer_->setTransform(transform, "unittest");
      }
    }

    // You can do set-up work for each test here.
  }

  virtual ~AngularVelocitySquareTest()
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp()
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown()
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case for AngularVelocity.
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  tf2::TimePoint tf2_time_;
};

TEST_F(LinearVelocitySquareTest, LinearVelocityToThreeFrames)
{
  std::vector<std::string> offset_frames;

  offset_frames.push_back("foo");
  offset_frames.push_back("stationary_offset_child");
  offset_frames.push_back("stationary_offset_parent");
  double epsilon = 1e-6;

  for (std::vector<std::string>::iterator it = offset_frames.begin(); it != offset_frames.end();
    ++it)
  {
    try {
      tf2::TimePoint check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 0.5);

      auto twist = buffer_->lookupVelocity(
        "bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, 1.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

      check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 1.5);
      twist = buffer_->lookupVelocity("bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, 1.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

      check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 2.5);
      twist = buffer_->lookupVelocity("bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, -1.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

      check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 3.5);
      twist = buffer_->lookupVelocity("bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, -1.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

      check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 4.5);
      twist = buffer_->lookupVelocity("bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, 1.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

      check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 5.5);
      twist = buffer_->lookupVelocity("bar", *it, check_time, tf2::durationFromSec(0.1));
      EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.linear.z, -1.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
      EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);
    } catch (tf2::TransformException & ex) {
      EXPECT_STREQ("", ex.what());
    }
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityAlone)
{
  double epsilon = 1e-6;
  try {
    tf2::TimePoint check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 0.5);
    auto twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 1.5);
    twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 2.5);
    twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 3.5);
    twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 4.5);
    twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 1.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 5.5);
    twist = buffer_->lookupVelocity("bar", "foo", check_time, tf2::durationFromSec(0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, -1.0, epsilon);
  } catch (tf2::TransformException & ex) {
    EXPECT_STREQ("", ex.what());
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityOffsetChildFrameInX)
{
  double epsilon = 1e-6;
  try {
    tf2::TimePoint check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 0.5);
    auto twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 1.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 2.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 3.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 4.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 1.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 5.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_child", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, -1.0, epsilon);
  } catch (tf2::TransformException & ex) {
    EXPECT_STREQ("", ex.what());
  }
}

TEST_F(AngularVelocitySquareTest, AngularVelocityOffsetParentFrameInZ)
{
  double epsilon = 1e-6;
  try {
    tf2::TimePoint check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 0.5);

    auto twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 1.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 2.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 3.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, -1.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 0.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 4.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, 1.0, epsilon);

    check_time = tf2::timeFromSec(tf2::timeToSec(tf2_time_) + 5.5);
    twist = buffer_->lookupVelocity(
      "bar", "stationary_offset_parent", check_time, tf2::durationFromSec(
        0.1));
    EXPECT_NEAR(twist.velocity.linear.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.linear.z, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.x, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.y, 0.0, epsilon);
    EXPECT_NEAR(twist.velocity.angular.z, -1.0, epsilon);
  } catch (tf2::TransformException & ex) {
    EXPECT_STREQ("", ex.what());
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
