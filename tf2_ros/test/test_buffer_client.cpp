/*
 * Copyright (c) 2019, Open Source Robotics Foundation, Inc.
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
 *     * Neither the name of the copyright holder nor the names of its
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
#include <future>
#include <thread>

#include <gtest/gtest.h>

#include <tf2_msgs/action/lookup_transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_client.h>

static const std::string ACTION_NAME = "test_tf2_buffer_action";

class MockBufferServer : public rclcpp::Node
{
  using LookupTransformAction = tf2_msgs::action::LookupTransform;
  using GoalHandle = rclcpp_action::ServerGoalHandle<LookupTransformAction>;

public:
  MockBufferServer()
    : rclcpp::Node("mock_buffer_server"),
      transform_available_(false)
  {
    action_server_ = rclcpp_action::create_server<LookupTransformAction>(
      get_node_base_interface(),
      get_node_clock_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      ACTION_NAME,
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const LookupTransformAction::Goal>)
        {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [](const std::shared_ptr<GoalHandle>){return rclcpp_action::CancelResponse::ACCEPT;},
      std::bind(&MockBufferServer::acceptedCallback, this, std::placeholders::_1));
  }

  void acceptedCallback(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Simulate doing some work in here; otherwise, we can complete the goal
    // before the rclcpp_action interface ever has time to do any work.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto result = std::make_shared<LookupTransformAction::Result>();
    if (transform_available_) {
      result->transform.transform = transform_;
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  void setTransform(const geometry_msgs::msg::Transform & transform)
  {
    transform_ = transform;
    transform_available_ = true;
  }

private:
  geometry_msgs::msg::Transform transform_;
  bool transform_available_;

  rclcpp_action::Server<LookupTransformAction>::SharedPtr action_server_;
};

class TestBufferClient : public ::testing::Test
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

  void SetUp()
  {
    node_ = std::make_shared<rclcpp::Node>("tf_buffer");
    client_ = std::make_unique<tf2_ros::BufferClient>(node_, ACTION_NAME);
    mock_server_ = std::make_shared<MockBufferServer>();

    executor_.add_node(node_);
    executor_.add_node(mock_server_);

    // Start spinning in a thread
    spin_thread_ = std::thread(
      std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor_));

    // Wait for discovery
    ASSERT_TRUE(client_->waitForServer(std::chrono::seconds(10)));
    // This hack ensure the action server can see the client
    // TODO(jacobperron): Replace with discovery rclcpp API when it exists
    while (mock_server_->count_subscribers("/" + ACTION_NAME + "/_action/feedback") < 1u) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void TearDown()
  {
    executor_.cancel();
    spin_thread_.join();
    mock_server_.reset();
    client_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::BufferClient> client_;
  std::shared_ptr<MockBufferServer> mock_server_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

TEST_F(TestBufferClient, lookup_transform_available)
{
  // Set a transform to be available
  geometry_msgs::msg::Transform transform_in;
  transform_in.rotation.w = 0.5;
  transform_in.translation.x = 42.0;
  mock_server_->setTransform(transform_in);

  auto transform_out = client_->lookupTransform(
    "test_target_frame", "test_source_frame", tf2::get_now());

  EXPECT_EQ(transform_out.transform.rotation.w, transform_in.rotation.w);
  EXPECT_EQ(transform_out.transform.translation.x, transform_in.translation.x);
}

TEST_F(TestBufferClient, lookup_transform_unavailable)
{
  EXPECT_THROW(
    client_->lookupTransform("test_target_frame", "test_source_frame", tf2::get_now()),
    tf2_ros::GoalAbortedException);
}

TEST_F(TestBufferClient, can_transform_available)
{
  // Set a transform to be available
  geometry_msgs::msg::Transform transform_in;
  transform_in.rotation.w = 0.5;
  transform_in.translation.x = 42.0;
  mock_server_->setTransform(transform_in);

  EXPECT_TRUE(client_->canTransform("test_target_frame", "test_source_frame", tf2::get_now()));
}

TEST_F(TestBufferClient, can_transform_unavailable)
{
  EXPECT_FALSE(client_->canTransform("test_target_frame", "test_source_frame", tf2::get_now()));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

