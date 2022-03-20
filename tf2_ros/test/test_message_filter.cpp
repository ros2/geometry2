/*
 * Copyright (c) 2022, Kenji Brameld
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

// This node is taken from https://docs.ros.org/en/foxy/Tutorials/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html#writing-the-message-filter-listener-node
class PoseDrawer : public rclcpp::Node
{
public:
  PoseDrawer()
  : Node("turtle_tf2_pose_drawer")
  {
    // Declare and acquire `target_frame` parameter
    this->declare_parameter<std::string>("target_frame", "turtle1");
    this->get_parameter("target_frame", target_frame_);

    typedef std::chrono::duration<int> seconds_type;
    seconds_type buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    point_sub_.subscribe(this, "/turtle3/turtle_point_stamped");
    tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
      point_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
      this->get_node_clock_interface(), buffer_timeout);
    // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
    tf2_filter_->registerCallback(&PoseDrawer::msgCallback, this);
  }

  bool msgCallbackCalled = false;

private:
  void msgCallback(const geometry_msgs::msg::PointStamped::SharedPtr)
  {
    msgCallbackCalled = true;
  }
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> tf2_filter_;
};

class TestMessageFilter : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestMessageFilter, test_callback_called)
{
  auto poseDrawerNode = std::make_shared<PoseDrawer>();
  auto testNode = std::make_shared<rclcpp::Node>("test_node");

  // Get current timestamp
  auto stamp = testNode->now();

  // Publish transform
  tf2_ros::TransformBroadcaster br(testNode);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "world";
  tf.child_frame_id = "turtle1";
  br.sendTransform(tf);

  // Publish pointstamped
  auto publisher = testNode->create_publisher<geometry_msgs::msg::PointStamped>(
    "/turtle3/turtle_point_stamped", 10);
  geometry_msgs::msg::PointStamped ps;
  ps.header.stamp = stamp;
  ps.header.frame_id = "world";
  publisher->publish(ps);

  rclcpp::spin_some(testNode);
  rclcpp::sleep_for(std::chrono::milliseconds(5));  // Wait for topics to come through?
  rclcpp::spin_some(poseDrawerNode);

  EXPECT_TRUE(poseDrawerNode->msgCallbackCalled);
}
