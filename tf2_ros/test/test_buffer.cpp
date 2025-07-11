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

#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <unordered_map>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/create_timer_interface.hpp"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/transform_listener.hpp"

class MockCreateTimer final : public tf2_ros::CreateTimerInterface
{
public:
  MockCreateTimer()
  : timer_handle_index_(0)
  {
  }

  tf2_ros::TimerHandle
  createTimer(
    rclcpp::Clock::SharedPtr clock,
    const tf2::Duration & period,
    tf2_ros::TimerCallbackType callback)
  {
    (void) clock;
    (void) period;
    const auto timer_handle = timer_handle_index_++;
    timer_to_callback_map_[timer_handle] = callback;
    return timer_handle;
  }

  void
  cancel(const tf2_ros::TimerHandle & timer_handle)
  {
    (void) timer_handle;
  }

  void
  reset(const tf2_ros::TimerHandle & timer_handle)
  {
    (void) timer_handle;
  }

  void
  remove(const tf2_ros::TimerHandle & timer_handle)
  {
    // Don't actually remove timer to avoid race condition
    (void) timer_handle;
  }

  void
  execute_timers()
  {
    for (const auto & elem : timer_to_callback_map_) {
      elem.second(elem.first);
    }
  }

  tf2_ros::TimerHandle timer_handle_index_;
  std::unordered_map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> timer_to_callback_map_;
};

class MockCreateTimerROS final : public tf2_ros::CreateTimerROS
{
public:
  MockCreateTimerROS(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers)
  : CreateTimerROS(node_base, node_timers), next_timer_handle_index_(0)
  {
  }

  tf2_ros::TimerHandle
  createTimer(
    rclcpp::Clock::SharedPtr clock,
    const tf2::Duration & period,
    tf2_ros::TimerCallbackType callback) override
  {
    auto timer_handle_index = next_timer_handle_index_++;
    auto timer_callback = std::bind(
      &MockCreateTimerROS::timerCallback, this, timer_handle_index,
      callback);
    timer_to_callback_map_[timer_handle_index] = timer_callback;
    return tf2_ros::CreateTimerROS::createTimer(clock, period, callback);
  }

  void
  execute_timers()
  {
    for (const auto & elem : timer_to_callback_map_) {
      elem.second(elem.first);
    }
  }

private:
  tf2_ros::TimerHandle next_timer_handle_index_;
  std::unordered_map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> timer_to_callback_map_;

  void
  timerCallback(
    const tf2_ros::TimerHandle & timer_handle,
    tf2_ros::TimerCallbackType callback)
  {
    callback(timer_handle);
  }
};

TEST(test_buffer, construct_with_null_clock)
{
  EXPECT_THROW(tf2_ros::Buffer(nullptr), std::invalid_argument);
}

TEST(test_buffer, can_transform_valid_transform)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);

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
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));

  auto output = buffer.lookupTransform("foo", "bar", tf2_time);
  EXPECT_STREQ(transform.child_frame_id.c_str(), output.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, output.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, output.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, output.transform.translation.z);

  auto output_rclcpp = buffer.lookupTransform("foo", "bar", rclcpp_time);
  EXPECT_STREQ(transform.child_frame_id.c_str(), output_rclcpp.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, output_rclcpp.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, output_rclcpp.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, output_rclcpp.transform.translation.z);
}

TEST(test_buffer, velocity_transform)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(
    rclcpp_time - rclcpp::Duration(0, static_cast<uint32_t>(1e+9)));
  transform.child_frame_id = "bar";
  transform.transform.translation.x = 0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(
    rclcpp_time + rclcpp::Duration(0, static_cast<uint32_t>(1e+9)));
  transform.child_frame_id = "bar";
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = 0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));

  geometry_msgs::msg::VelocityStamped output =
    buffer.lookupVelocity("bar", "foo", tf2_time, tf2::durationFromSec(0.1));

  output =
    buffer.lookupVelocity(
    "bar", "foo",
    "bar", {0, 0, 0}, "bar",
    tf2_time, tf2::durationFromSec(0.1));

  double epsilon = 1e-6;
  EXPECT_NEAR(output.velocity.linear.x, 1.0, epsilon);
  EXPECT_NEAR(output.velocity.linear.y, 0.0, epsilon);
  EXPECT_NEAR(output.velocity.linear.z, 0.0, epsilon);
  EXPECT_NEAR(output.velocity.angular.x, 0.0, epsilon);
  EXPECT_NEAR(output.velocity.angular.y, 0.0, epsilon);
  EXPECT_NEAR(output.velocity.angular.z, 0.0, epsilon);
}


TEST(test_buffer, test_twist)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  float vel = 0.3f;
  for (int i = -10; i < 5; ++i) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "PARENT";
    if (i < 0) {
      transform.header.stamp =
        builtin_interfaces::msg::Time(rclcpp_time - rclcpp::Duration(
          static_cast<int32_t>(std::fabs(i)), 0));
    } else {
      transform.header.stamp = builtin_interfaces::msg::Time(rclcpp_time + rclcpp::Duration(i, 0));
    }
    transform.child_frame_id = "THISFRAME";
    transform.transform.translation.x = i * vel;
    transform.transform.translation.y = 0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    EXPECT_TRUE(buffer.setTransform(transform, "unittest"));
  }

  auto tw0 = buffer.lookupVelocity("THISFRAME", "PARENT", tf2_time, tf2::durationFromSec(4.001));

  auto tw1 = buffer.lookupVelocity(
    "THISFRAME", "PARENT", "PARENT", {0, 0, 0}, "THISFRAME",
    tf2_time, tf2::durationFromSec(4.001));

  double epsilon = 1e-6;
  EXPECT_NEAR(tw1.velocity.linear.x, 0.3, epsilon);
  EXPECT_NEAR(tw1.velocity.linear.y, 0.0, epsilon);
  EXPECT_NEAR(tw1.velocity.linear.z, 0.0, epsilon);
  EXPECT_NEAR(tw1.velocity.angular.x, 0.0, epsilon);
  EXPECT_NEAR(tw1.velocity.angular.y, 0.0, epsilon);
  EXPECT_NEAR(tw1.velocity.angular.z, 0.0, epsilon);
}

TEST(test_buffer, can_transform_without_dedicated_thread)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(false);

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

  // Should NOT error with default timeout
  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));
  // Should error when timeout is not default
  EXPECT_FALSE(buffer.canTransform("bar", "foo", tf2_time, std::chrono::seconds(2)));
  EXPECT_FALSE(buffer.canTransform("bar", "foo", rclcpp_time, rclcpp::Duration::from_seconds(1.0)));

  auto output = buffer.lookupTransform("foo", "bar", tf2_time);
  EXPECT_STREQ(transform.child_frame_id.c_str(), output.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, output.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, output.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, output.transform.translation.z);

  auto output_rclcpp = buffer.lookupTransform("foo", "bar", rclcpp_time);
  EXPECT_STREQ(transform.child_frame_id.c_str(), output_rclcpp.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, output_rclcpp.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, output_rclcpp.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, output_rclcpp.transform.translation.z);
}

TEST(test_buffer, wait_for_transform_valid)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);
  auto mock_create_timer = std::make_shared<MockCreateTimer>();
  buffer.setCreateTimerInterface(mock_create_timer);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  geometry_msgs::msg::TransformStamped transform_callback_result;
  auto future = buffer.waitForTransform(
    "foo",
    "bar",
    tf2_time, tf2::durationFromSec(1.0),
    [&transform_callback_result](const tf2_ros::TransformStampedFuture & future)
    {
      transform_callback_result = future.get();
    });

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(rclcpp_time);
  transform.child_frame_id = "bar";
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.translation.z = 3.0;
  transform.transform.rotation.w = 1.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;

  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));
  const auto status = future.wait_for(std::chrono::seconds(1));
  EXPECT_EQ(status, std::future_status::ready);

  auto transform_result = future.get();
  EXPECT_STREQ(transform.child_frame_id.c_str(), transform_result.child_frame_id.c_str());
  EXPECT_STREQ(transform.child_frame_id.c_str(), transform_callback_result.child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, transform_result.transform.translation.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, transform_result.transform.translation.y);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, transform_result.transform.translation.z);
  EXPECT_DOUBLE_EQ(
    transform.transform.translation.x,
    transform_callback_result.transform.translation.x);
  EXPECT_DOUBLE_EQ(
    transform.transform.translation.y,
    transform_callback_result.transform.translation.y);
  EXPECT_DOUBLE_EQ(
    transform.transform.translation.z,
    transform_callback_result.transform.translation.z);

  // Expect there to be exactly one timer
  EXPECT_EQ(mock_create_timer->timer_to_callback_map_.size(), 1u);
}

TEST(test_buffer, wait_for_transform_timeout)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);
  auto mock_create_timer = std::make_shared<MockCreateTimer>();
  buffer.setCreateTimerInterface(mock_create_timer);

  rclcpp::Time time_start = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(time_start.nanoseconds()));

  bool callback_timeout = false;
  auto future = buffer.waitForTransform(
    "foo",
    "bar",
    tf2_time, tf2::durationFromSec(1.0),
    [&callback_timeout](const tf2_ros::TransformStampedFuture & future)
    {
      try {
        // Expect this to throw an exception due to timeout
        future.get();
      } catch (...) {
        callback_timeout = true;
      }
    });

  // Set an irrelevant transform
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "test";
  transform.header.stamp = builtin_interfaces::msg::Time(clock->now());
  transform.child_frame_id = "baz";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  auto status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::timeout);

  // Fake a time out
  mock_create_timer->execute_timers();

  EXPECT_FALSE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_FALSE(buffer.canTransform("bar", "foo", time_start));
  status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::ready);
  EXPECT_TRUE(callback_timeout);
}

// Regression test for https://github.com/ros2/geometry2/issues/141
TEST(test_buffer, wait_for_transform_race)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);
  auto mock_create_timer = std::make_shared<MockCreateTimer>();
  buffer.setCreateTimerInterface(mock_create_timer);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  bool callback_timeout = false;
  auto future = buffer.waitForTransform(
    "foo",
    "bar",
    tf2_time, tf2::durationFromSec(1.0),
    [&callback_timeout](const tf2_ros::TransformStampedFuture & future)
    {
      try {
        // We don't expect this throw, even though a timeout will occur
        future.get();
      } catch (...) {
        callback_timeout = true;
      }
    });

  auto status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::timeout);

  // Set the valid transform during the timeout
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(rclcpp_time);
  transform.child_frame_id = "bar";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  // Fake a time out (race with setTransform above)
  mock_create_timer->execute_timers();

  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));
  status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::ready);
  EXPECT_FALSE(callback_timeout);
}

TEST(test_buffer, timer_ros_wait_for_transform_race)
{
  int argc = 1;
  char const * const argv[] = {"timer_ros_wait_for_transform_race"};
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> rclcpp_node_ = std::make_shared<rclcpp::Node>(
    "timer_ros_wait_for_transform_race");

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  // Silence error about dedicated thread's being necessary
  buffer.setUsingDedicatedThread(true);
  auto mock_create_timer_ros = std::make_shared<MockCreateTimerROS>(
    rclcpp_node_->get_node_base_interface(),
    rclcpp_node_->get_node_timers_interface());
  buffer.setCreateTimerInterface(mock_create_timer_ros);

  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  bool callback_timeout = false;
  auto future = buffer.waitForTransform(
    "foo",
    "bar",
    tf2_time, tf2::durationFromSec(1.0),
    [&callback_timeout](const tf2_ros::TransformStampedFuture & future)
    {
      try {
        // We don't expect this throw, even though a timeout will occur
        future.get();
      } catch (...) {
        callback_timeout = true;
      }
    });

  auto status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::timeout);

  // Set the valid transform during the timeout
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "foo";
  transform.header.stamp = builtin_interfaces::msg::Time(rclcpp_time);
  transform.child_frame_id = "bar";
  transform.transform.rotation.w = 1.0;
  EXPECT_TRUE(buffer.setTransform(transform, "unittest"));

  // Fake a time out (race with setTransform above)
  EXPECT_NO_THROW(mock_create_timer_ros->execute_timers());

  EXPECT_TRUE(buffer.canTransform("bar", "foo", tf2_time));
  EXPECT_TRUE(buffer.canTransform("bar", "foo", rclcpp_time));
  status = future.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status, std::future_status::ready);
  EXPECT_FALSE(callback_timeout);
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
