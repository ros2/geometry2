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

/** \author Josh Faust */

#include <gtest/gtest.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <functional>
#include <memory>

class Notification
{
public:
  Notification(int expected_count) :
    count_(0), expected_count_(expected_count), failure_count_(0)
  {
  }

  void notify(const geometry_msgs::msg::PointStamped::ConstSharedPtr& message)
  {
    ++count_;
  }

  void failure(const geometry_msgs::msg::PointStamped::ConstSharedPtr& message, tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    ++failure_count_;
  }

  int count_;
  int expected_count_;
  int failure_count_;
};

TEST(MessageFilter, noTransforms)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);
  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_);
}

TEST(MessageFilter, noTransformsSameFrame)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);
  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
  msg->header.frame_id = "frame1";
  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

geometry_msgs::msg::TransformStamped createTransform(tf2::Quaternion q, tf2::Vector3 v, builtin_interfaces::msg::Time stamp, const std::string& frame1, const std::string& frame2)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = frame1;
  t.child_frame_id = frame2;
  t.header.stamp = stamp;
  t.transform.translation.x = v.x();
  t.transform.translation.y = v.y();
  t.transform.translation.z = v.z();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

TEST(MessageFilter, preexistingTransforms)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);

  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  builtin_interfaces::msg::Time stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, postTransforms)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);

  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  builtin_interfaces::msg::Time stamp = tf2_ros::toMsg(tf2::timeFromSec(1));

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(0, n.count_);

  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_);
}
// TODO (ahcorde): For some unknown reason message_filters::Connection registerFailureCallback is disable
// with #if 0 https://github.com/ros2/geometry2/blob/ros2/tf2_ros/include/tf2_ros/message_filter.h#L463
// rework this part when this is available
// TEST(MessageFilter, queueSize)
// {
//   auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
//   auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
//     node->get_node_base_interface(),
//     node->get_node_timers_interface());
//
//   rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
//   tf2_ros::Buffer buffer(clock);
//   buffer.setCreateTimerInterface(create_timer_interface);
//   tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
//   Notification n(10);
//
//   filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));
//   // filter.registerFailureCallback(std::bind(&Notification::failure, &n,  std::placeholders::_1,  std::placeholders::_2));
//
//   builtin_interfaces::msg::Time stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
//
//   for (int i = 0; i < 20; ++i)
//   {
//     std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
//     msg->header.stamp = stamp;
//     msg->header.frame_id = "frame2";
//
//     filter.add(msg);
//   }
//
//   EXPECT_EQ(0, n.count_);
//   EXPECT_EQ(10, n.failure_count_);
//
//   buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame2"), "me");
//
//   EXPECT_EQ(10, n.count_);
//
//
// }

TEST(MessageFilter, setTargetFrame)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);
  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));
  filter.setTargetFrame("frame1000");

  builtin_interfaces::msg::Time stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1000", "frame2"), "me");

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";

  filter.add(msg);

  EXPECT_EQ(1, n.count_);
}

TEST(MessageFilter, multipleTargetFrames)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "", 10, node);
  Notification n(1);
  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  std::vector<std::string> target_frames;
  target_frames.push_back("frame1");
  target_frames.push_back("frame2");
  filter.setTargetFrames(target_frames);

  builtin_interfaces::msg::Time stamp = tf2_ros::toMsg(tf2::timeFromSec(1));
  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame3"), "me");

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame3";
  filter.add(msg);

  EXPECT_EQ(0, n.count_); // frame1->frame3 exists, frame2->frame3 does not (yet)

  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_); // frame2->frame3 now exists
}

TEST(MessageFilter, tolerance)
{
  auto node = rclcpp::Node::make_shared("tf2_ros_message_filter");
  auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node->get_node_base_interface(),
    node->get_node_timers_interface());

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setCreateTimerInterface(create_timer_interface);
  tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped> filter(buffer, "frame1", 10, node);
  Notification n(1);
  filter.registerCallback(std::bind(&Notification::notify, &n, std::placeholders::_1));

  tf2::Duration offset = tf2::durationFromSec(0.2);
  filter.setTolerance(offset);

  builtin_interfaces::msg::Time stamp = rclcpp::Time(1, 0);
  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp, "frame1", "frame2"), "me");

  std::shared_ptr<geometry_msgs::msg::PointStamped> msg = std::make_shared<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "frame2";
  filter.add(msg);

  EXPECT_EQ(0, n.count_); //No return due to lack of space for offset

  double time_stamp = (stamp.sec + stamp.nanosec/1e9) + tf2::durationToSec(offset)*1.1;
  builtin_interfaces::msg::Time stamp_transform = rclcpp::Time(static_cast<int32_t>((int)time_stamp), static_cast<uint32_t>((time_stamp - (int)time_stamp)*1e9));

  buffer.setTransform(createTransform(tf2::Quaternion(0,0,0,1), tf2::Vector3(1,2,3), stamp_transform, "frame1", "frame2"), "me");

  EXPECT_EQ(1, n.count_); // Now have data for the message published earlier

  time_stamp = (stamp.sec + stamp.nanosec/1e9) + tf2::durationToSec(offset);

  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(time_stamp));
  filter.add(msg);

  EXPECT_EQ(1, n.count_); // Latest message is off the end of the offset
}

// TODO(ahcorde): For some unknown reason message_filters::Connection registerFailureCallback is disable
// with #if 0 https://github.com/ros2/geometry2/blob/ros2/tf2_ros/include/tf2_ros/message_filter.h#L463
// rework this part when this is available

// TEST(MessageFilter, outTheBackFailure)
// {
//   BufferCore bc;
//   Notification n(1);
//   MessageFilter<geometry_msgs::msg::PointStamped> filter(bc, "frame1", 1, 0);
//   filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));
//
//   builtin_interfaces::msg::Time stamp(1);
//   bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp, "frame1", "frame2"), "me");
//   bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp + tf2::durationFromSec(10000), "frame1", "frame2"), "me");
//
//   geometry_msgs::msg::PointStampedPtr msg(new geometry_msgs::msg::PointStamped);
//   msg->header.stamp = stamp;
//   msg->header.frame_id = "frame2";
//   filter.add(msg);
//
//   EXPECT_EQ(1, n.failure_count_);
// }
//
// TEST(MessageFilter, outTheBackFailure2)
// {
//   BufferCore bc;
//   Notification n(1);
//   MessageFilter<geometry_msgs::msg::PointStamped> filter(bc, "frame1", 1, 0);
//   filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));
//
//   builtin_interfaces::msg::Time stamp(1);
//
//   geometry_msgs::msg::PointStampedPtr msg(new geometry_msgs::msg::PointStamped);
//   msg->header.stamp = stamp;
//   msg->header.frame_id = "frame2";
//   filter.add(msg);
//
//   EXPECT_EQ(0, n.count_);
//   EXPECT_EQ(0, n.failure_count_);
//
//   bc.setTransform(createTransform(Quaternion(0,0,0,1), Vector3(1,2,3), stamp + tf2::durationFromSec(10000), "frame1", "frame2"), "me");
//
//   EXPECT_EQ(1, n.failure_count_);
// }
//
// TEST(MessageFilter, emptyFrameIDFailure)
// {
//   BufferCore bc;
//   Notification n(1);
//   MessageFilter<geometry_msgs::msg::PointStamped> filter(bc, "frame1", 1, 0);
//   filter.registerFailureCallback(boost::bind(&Notification::failure, &n, _1, _2));
//
//   geometry_msgs::msg::PointStampedPtr msg(new geometry_msgs::msg::PointStamped);
//   msg->header.frame_id = "";
//   filter.add(msg);
//
//   EXPECT_EQ(1, n.failure_count_);
// }
//
// TEST(MessageFilter, callbackQueue)
// {
//   BufferCore bc;
//   Notification n(1);
//   ros::CallbackQueue queue;
//   MessageFilter<geometry_msgs::msg::PointStamped> filter(bc, "frame1", 1, &queue);
//   filter.registerCallback(boost::bind(&Notification::notify, &n, _1));
//
//   geometry_msgs::msg::PointStampedPtr msg(new geometry_msgs::msg::PointStamped);
//   msg->header.stamp = builtin_interfaces::msg::Time(1);
//   msg->header.frame_id = "frame1";
//   filter.add(msg);
//
//   EXPECT_EQ(0, n.count_);
//
//   queue.callAvailable();
//
//   EXPECT_EQ(1, n.count_);
// }


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
